import math
import numpy as np
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, PoseArray, Quaternion, Vector3Stamped
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_multiply

# ----------- Helpers (math + quaternion conversions) ------------
def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        return v
    return v / n

def rotation_from_frame(forward: np.ndarray, up: np.ndarray) -> np.ndarray:
    # Building rotation matrix with given forward and up vectors. Should return a 3x3 rotation matrix.

    f = normalize(forward)
    u = normalize(up)
    # Recompute orthonormal basis: x = f, z = component orthogonal to f of u, y = z x x
    z = u - np.dot(u, f) * f
    if np.linalg.norm(z) < 1e-6:
        # fallback: make z perpendicular
        if abs(f[2]) < 0.9:
            z = np.cross(f, np.array([0, 0, 1.0]))
        else:
            z = np.cross(f, np.array([0, 1.0, 0]))
    z = normalize(z)
    y = np.cross(z, f)
    y = normalize(y)
    R = np.column_stack((f, y, z))  # columns are x,y,z
    return R

def quat_from_rotation_matrix(R: np.ndarray) -> np.ndarray:

    # Converting our 3x3 rotation matrix to quaternion (x,y,z,w) using tf_transformations.quaternion_from_matrix. Should expect 4x4.
    M = np.eye(4)
    M[:3, :3] = R
    q = quaternion_from_matrix(M)
    # quaternion_from_matrix returns [x,y,z,w]
    return q

def pose_from_position_and_quat(position: np.ndarray, quat: np.ndarray) -> Pose:
    p = Pose()
    p.position.x = float(position[0]); p.position.y = float(position[1]); p.position.z = float(position[2])
    p.orientation = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
    return p

# Simple Catmull-Rom spline helper
def catmull_rom_spline(p0, p1, p2, p3, t):
    # Return point for parameter t in [0,1] on Catmull-Rom between p1->p2
    t2 = t * t
    t3 = t2 * t
    return 0.5 * ((2 * p1) +
                  (-p0 + p2) * t +
                  (2*p0 - 5*p1 + 4*p2 - p3) * t2 +
                  (-p0 + 3*p1 - 3*p2 + p3) * t3)


# -------------------- Node --------------------
class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__("waypoint_manager_node")

        # -------- Params (tunable) --------
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("orientation_mode", "directional")  # directional | fixed | incoming
        self.declare_parameter("tool_tilt_deg", 15.0)               # tilt relative to forward (degrees)
        self.declare_parameter("path_mode", "straight")            # straight | curved
        self.declare_parameter("interpolation_resolution", 0.02)   # meters between poses on generated path
        self.declare_parameter("min_normal_points", 3)             # points required to compute normal

        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.orientation_mode = self.get_parameter("orientation_mode").get_parameter_value().string_value
        self.tool_tilt_deg = float(self.get_parameter("tool_tilt_deg").get_parameter_value().double_value)
        self.path_mode = self.get_parameter("path_mode").get_parameter_value().string_value
        self.interp_res = float(self.get_parameter("interpolation_resolution").get_parameter_value().double_value)
        self.min_normal_points = int(self.get_parameter("min_normal_points").get_parameter_value().integer_value)

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal waypoint storage (poses in base_frame)
        self.waypoints: List[PoseStamped] = []

        # Subscribers: clicked points (PointStamped) and incoming PoseStamped (local/tool-space)
        self.create_subscription(PointStamped, "/clicked_point", self.clicked_point_cb, 10)
        self.create_subscription(PoseStamped, "/local_waypoint", self.local_waypoint_cb, 10)

        # Publisher: PoseArray with TRANSIENT_LOCAL durability (late subscribers get the last trajectory)
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.posearray_pub = self.create_publisher(PoseArray, "/weld_path", qos)

        # Services: clear and force-publish
        self.clear_srv = self.create_service(Empty, "clear_waypoints", self.handle_clear)
        self.publish_srv = self.create_service(Empty, "publish_waypoints", self.handle_publish_request)

        self.get_logger().info("WaypointManagerNode started. mode=%s path=%s", self.orientation_mode, self.path_mode)

    # ----- Callbacks -----
    def clicked_point_cb(self, msg: PointStamped):
        # Receive a clicked point, transform to base_frame, store and optionally auto-publish.
        try:
            trans = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed for clicked point: {e}")
            return

        p = self.transform_pointstamped(msg, trans)
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = p[0]; ps.pose.position.y = p[1]; ps.pose.position.z = p[2]
        # Orientation will be set later during path generation
        self.waypoints.append(ps)
        self.get_logger().info("Added waypoint (clicked) in %s: [%.3f, %.3f, %.3f]", self.base_frame, p[0], p[1], p[2])

    def local_waypoint_cb(self, msg: PoseStamped):
        # Receive a PoseStamped in some frame, transform pose into base_frame and store.
        try:
            trans = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF lookup failed for local waypoint: {e}")
            return

        ps = self.transform_posestamped(msg, trans)
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        self.waypoints.append(ps)
        self.get_logger().info("Added waypoint (pose) in %s: [%.3f, %.3f, %.3f]", self.base_frame,
                               ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)

    # ----- Service handlers -----
    def handle_clear(self, request, response):
        self.waypoints.clear()
        self.get_logger().info("Cleared all waypoints.")
        return response

    def handle_publish_request(self, request, response):
        self.publish_weld_path()
        return response

    # ----- Path generation & publishing -----
    def publish_weld_path(self):
        if len(self.waypoints) == 0:
            self.get_logger().warn("No waypoints to publish.")
            return

        # Gather positions as numpy arrays
        positions = [np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]) for p in self.waypoints]

        # Generate list of positions for the final path
        if self.path_mode == "straight" or len(positions) < 4:
            path_positions = self.generate_straight_positions(positions, self.interp_res)
        else:
            path_positions = self.generate_curved_positions(positions, self.interp_res)

        # Compute pose orient for each position
        poses = []
        for idx, pos in enumerate(path_positions):
            # direction: use next point or previous if last
            if idx < len(path_positions) - 1:
                forward = path_positions[idx + 1] - pos
            else:
                forward = pos - path_positions[idx - 1] if idx > 0 else np.array([1.0, 0.0, 0.0])

            forward = normalize(forward)
            # compute surface normal if possible
            normal = self.estimate_surface_normal_around(pos, positions) if len(positions) >= self.min_normal_points else np.array([0.0, 0.0, 1.0])
            if np.linalg.norm(normal) < 1e-6:
                normal = np.array([0.0, 0.0, 1.0])

            # orientation depending on mode
            if self.orientation_mode == "fixed":
                # Fixed pointing downwards but with tool tilt applied
                base_quat = quaternion_from_euler(0.0, math.pi, 0.0)  # example
            elif self.orientation_mode == "incoming":
                # Try to use nearest incoming orientation (not implemented here)
                # fallback to directional
                base_quat = self.compute_orientation_quat(forward, normal, self.tool_tilt_deg)
            else:
                # default: directional aligned with forward + surface normal
                base_quat = self.compute_orientation_quat(forward, normal, self.tool_tilt_deg)

            pose_msg = pose_from_position_and_quat(pos, base_quat)
            poses.append(pose_msg)

        pa = PoseArray()
        pa.header.frame_id = self.base_frame
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.poses = poses

        self.posearray_pub.publish(pa)
        self.get_logger().info("Published weld path: %d poses (mode=%s, orientation=%s)", len(poses), self.path_mode, self.orientation_mode)

    # ----- Position generation functions -----
    def generate_straight_positions(self, positions: List[np.ndarray], resolution: float) -> List[np.ndarray]:
        out = []
        for i in range(len(positions) - 1):
            a = positions[i]; b = positions[i + 1]
            seg = b - a
            dist = np.linalg.norm(seg)
            if dist < 1e-9:
                continue
            steps = max(1, int(math.ceil(dist / resolution)))
            for s in range(steps):
                t = s / float(steps)
                out.append(a * (1.0 - t) + b * t)
        out.append(positions[-1])
        return out

    def generate_curved_positions(self, positions: List[np.ndarray], resolution: float) -> List[np.ndarray]:
        # Catmull-Rom spline: require adding endpoints for p0 and p_{n+1}
        pts = positions
        n = len(pts)
        if n < 4:
            return self.generate_straight_positions(positions, resolution)

        out = []
        # For endpoints, repeat first/last to create p0/pn+1
        extended = [pts[0]] + pts + [pts[-1]]
        for i in range(0, n - 1):
            p0 = extended[i]
            p1 = extended[i + 1]
            p2 = extended[i + 2]
            p3 = extended[i + 3]
            # estimate segment length to determine sampling count
            seg_len = np.linalg.norm(p2 - p1)
            if seg_len < 1e-9:
                continue
            steps = max(1, int(math.ceil(seg_len / resolution)))
            for s in range(steps):
                t = s / float(steps)
                pt = catmull_rom_spline(p0, p1, p2, p3, t)
                out.append(pt)
        out.append(pts[-1])
        return out

    # ----- Orientation & normals -----
    def compute_orientation_quat(self, forward: np.ndarray, normal: np.ndarray, tilt_deg: float) -> np.ndarray:
        
        #Compose quaternion such that:
        # - X axis (forward) aligns with weld direction
        # - Z axis aligns with surface normal (as close as possible)
        # - apply a fixed tilt (tilt_deg) around the Y axis (tool tilt)
        #Returns quaternion as [x,y,z,w]
        
        R = rotation_from_frame(forward, normal)
        q = quat_from_rotation_matrix(R)  # x,y,z,w

        # Apply tilt around local Y axis by composing with a small rotation
        tilt_rad = math.radians(tilt_deg)
        q_tilt = quaternion_from_euler(0.0, tilt_rad, 0.0)  # (roll, pitch, yaw) applied in tool frame
        q_final = quaternion_multiply(q, q_tilt)  # q * q_tilt
        return q_final

    def estimate_surface_normal_around(self, sample_pos: np.ndarray, base_positions: List[np.ndarray]) -> np.ndarray:
        
        #Simple normal estimate: find k nearest base_positions to sample_pos and compute PCA (or cross-product)
        #Using cross product of vectors between neighbors to make a quick approximation
        
        # find k nearest
        pts = np.array(base_positions)
        dists = np.linalg.norm(pts - sample_pos, axis=1)
        k = min(6, len(pts))
        idx = np.argsort(dists)[:k]
        if len(idx) < 3:
            return np.array([0.0, 0.0, 1.0])

        neighbors = pts[idx]
        # center
        c = np.mean(neighbors, axis=0)
        cov = np.cov((neighbors - c).T)
        # Eigenvector corresponding to smallest eigenvalue is normal
        w, v = np.linalg.eig(cov)
        min_idx = int(np.argmin(w))
        normal = v[:, min_idx]
        normal = normalize(normal.real)
        # ensure upward-ish normal (z positive) to avoid flipping
        if normal[2] < 0:
            normal = -normal
        return normal

    # ----- TF transform helpers (point/pose) -----
    def transform_pointstamped(self, p: PointStamped, tf) -> np.ndarray:
        # Apply translation + rotation from transform to point. tf is a geometry_msgs/TransformStamped.
        # transform translation
        tx = tf.transform.translation.x; ty = tf.transform.translation.y; tz = tf.transform.translation.z
        # rotation quaternion
        q = tf.transform.rotation
        # convert quaternion to rotation matrix
        qmat = quaternion_from_euler(0, 0, 0)  # placeholder - we'll use matrix method instead
        # easier: use quaternion_from_matrix method by building a 4x4 transform matrix
        R = np.eye(4)
        # but tf gives quaternion, we build rotation 3x3 from quaternion directly
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        # rotation matrix elements
        R3 = np.array([
            [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),         2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ])
        pt = np.array([p.point.x, p.point.y, p.point.z])
        transformed = R3.dot(pt) + np.array([tx, ty, tz])
        return transformed

    def transform_posestamped(self, ps: PoseStamped, tf) -> PoseStamped:
        # Apply transform (TransformStamped) to PoseStamped (approximate: rotate orientation and translate position).

        # Use the same math as the point transform for translation; rotate quaternion
        tx = tf.transform.translation.x; ty = tf.transform.translation.y; tz = tf.transform.translation.z
        q = tf.transform.rotation
        # rotation matrix from quaternion
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        R3 = np.array([
            [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),         2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ])
        pt = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
        transformed_pos = R3.dot(pt) + np.array([tx, ty, tz])

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.pose.position.x = float(transformed_pos[0])
        out.pose.position.y = float(transformed_pos[1])
        out.pose.position.z = float(transformed_pos[2])

        # For orientation: multiply transform rotation * incoming pose orientation
        # incoming quaternion
        iq = ps.pose.orientation
        iq_arr = np.array([iq.x, iq.y, iq.z, iq.w])
        tq = np.array([q.x, q.y, q.z, q.w])
        # combined = tq * iq (quaternion multiplication)
        combined = quaternion_multiply(tq, iq_arr)
        out.pose.orientation = Quaternion(x=combined[0], y=combined[1], z=combined[2], w=combined[3])

        return out


# -------------------- Main --------------------
def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()

    try:
        # Auto-publish on a timer if you want periodic updates (optional)
        publish_period = 0.5  # seconds
        node.create_timer(publish_period, node.publish_weld_path)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down waypoint manager.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
