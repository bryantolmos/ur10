import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy

import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose, Vector3
from visualization_msgs.msg import Marker
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA


class ObjectPublisherNode(Node):
    """
    Publishes TF, Marker, and CollisionObject for a known object at a fixed pose,
    and adds a static floor collision object.
    Publishes a few times at startup, then stops.
    """
    def __init__(self):
        super().__init__('object_publisher_node')

        # --- Parameters (same as original) ---
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('object_frame', 'known_object_frame')
        self.declare_parameter('pose.position.x', 0.5)
        self.declare_parameter('pose.position.y', 0.0)
        self.declare_parameter('pose.position.z', 0.03)
        self.declare_parameter('pose.orientation.x', 0.0)
        self.declare_parameter('pose.orientation.y', 0.0)
        self.declare_parameter('pose.orientation.z', 0.0)
        self.declare_parameter('pose.orientation.w', 1.0)
        self.declare_parameter('dimensions', [0.1, 0.5, 0.36])
        self.declare_parameter('collision_padding', 0.0)
        self.declare_parameter('publish_rate_hz', 1.0)
        # NEW: how many times to publish before stopping
        self.declare_parameter('max_publishes', 10)

        self.declare_parameter('floor.enable', True)
        self.declare_parameter('floor.pose.position.x', 0.0)
        self.declare_parameter('floor.pose.position.y', 0.0)
        self.declare_parameter('floor.pose.position.z', -0.02)
        self.declare_parameter('floor.pose.orientation.w', 1.0)
        self.declare_parameter('floor.size', [2.0, 2.0, 0.01])

        # --- Get params ---
        self.parent_frame_ = self.get_parameter('parent_frame').value
        self.object_frame_ = self.get_parameter('object_frame').value
        self.object_dims_ = self.get_parameter('dimensions').value
        self.collision_padding_ = self.get_parameter('collision_padding').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self.max_publishes_ = int(self.get_parameter('max_publishes').value)

        self.object_pose_ = Pose()
        self.object_pose_.position.x = self.get_parameter('pose.position.x').value
        self.object_pose_.position.y = self.get_parameter('pose.position.y').value
        self.object_pose_.position.z = self.get_parameter('pose.position.z').value
        self.object_pose_.orientation.x = self.get_parameter('pose.orientation.x').value
        self.object_pose_.orientation.y = self.get_parameter('pose.orientation.y').value
        self.object_pose_.orientation.z = self.get_parameter('pose.orientation.z').value
        self.object_pose_.orientation.w = self.get_parameter('pose.orientation.w').value

        self.publish_floor_ = self.get_parameter('floor.enable').value
        self.floor_pose_ = Pose()
        self.floor_pose_.position.x = self.get_parameter('floor.pose.position.x').value
        self.floor_pose_.position.y = self.get_parameter('floor.pose.position.y').value
        self.floor_pose_.position.z = self.get_parameter('floor.pose.position.z').value
        self.floor_pose_.orientation.x = 0.0
        self.floor_pose_.orientation.y = 0.0
        self.floor_pose_.orientation.z = 0.0
        self.floor_pose_.orientation.w = self.get_parameter('floor.pose.orientation.w').value
        self.floor_dims_ = self.get_parameter('floor.size').value

        self.get_logger().info(
            f"Known object configured in frame '{self.object_frame_}' relative to '{self.parent_frame_}'"
        )
        self.get_logger().info(f"Object dims: {list(self.object_dims_)}")
        if self.publish_floor_:
            self.get_logger().info(
                f"Floor enabled at Z={self.floor_pose_.position.z:.3f}, size={list(self.floor_dims_)}"
            )

        # Static TF
        self.tf_static_broadcaster_ = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_tf()

        # Publishers (latched)
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub_ = self.create_publisher(Marker, '/known_object_marker', latching_qos)
        self.collision_pub_ = self.create_publisher(CollisionObject, '/collision_object', latching_qos)
        self.floor_marker_pub_ = self.create_publisher(Marker, '/floor_marker', latching_qos)

        # Timer that publishes a few times, then stops
        self.publish_count_ = 0
        if publish_rate > 0:
            self.timer_ = self.create_timer(1.0 / publish_rate, self.timer_callback)
            self.get_logger().info(
                f"Publishing visuals at {publish_rate} Hz for up to {self.max_publishes_} messages."
            )
        else:
            # Fallback: publish once if rate=0
            self.publish_all_objects()
            self.get_logger().info("Published visuals once (publish_rate_hz=0).")

    def timer_callback(self):
        self.publish_all_objects()
        self.publish_count_ += 1
        if self.publish_count_ >= self.max_publishes_:
            self.get_logger().info("Reached max_publishes, stopping periodic publishes.")
            self.timer_.cancel()

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame_
        t.child_frame_id = self.object_frame_
        t.transform.translation.x = self.object_pose_.position.x
        t.transform.translation.y = self.object_pose_.position.y
        t.transform.translation.z = self.object_pose_.position.z
        t.transform.rotation.x = self.object_pose_.orientation.x
        t.transform.rotation.y = self.object_pose_.orientation.y
        t.transform.rotation.z = self.object_pose_.orientation.z
        t.transform.rotation.w = self.object_pose_.orientation.w
        self.tf_static_broadcaster_.sendTransform(t)

    def publish_all_objects(self):
        now_msg = self.get_clock().now().to_msg()
        self.publish_target_object(now_msg)
        if self.publish_floor_:
            self.publish_floor_object(now_msg)

    def publish_target_object(self, stamp):
        marker = Marker()
        marker.header.frame_id = self.parent_frame_
        marker.header.stamp = stamp
        marker.ns = "known_object"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self.object_pose_
        marker.scale = Vector3(
            x=self.object_dims_[0],
            y=self.object_dims_[1],
            z=self.object_dims_[2]
        )
        marker.color = ColorRGBA(r=0.3, g=0.3, b=0.8, a=0.7)
        marker.lifetime = Duration(seconds=0).to_msg()
        self.marker_pub_.publish(marker)

        co = CollisionObject()
        co.header.frame_id = self.parent_frame_
        co.header.stamp = stamp
        co.id = "known_target_object"
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            self.object_dims_[0] + 2 * self.collision_padding_,
            self.object_dims_[1] + 2 * self.collision_padding_,
            self.object_dims_[2] + 2 * self.collision_padding_
        ]
        co.primitives.append(primitive)
        co.primitive_poses.append(self.object_pose_)
        self.collision_pub_.publish(co)

    def publish_floor_object(self, stamp):
        floor_marker = Marker()
        floor_marker.header.frame_id = self.parent_frame_
        floor_marker.header.stamp = stamp
        floor_marker.ns = "floor"
        floor_marker.id = 0
        floor_marker.type = Marker.CUBE
        floor_marker.action = Marker.ADD
        floor_marker.pose = self.floor_pose_
        floor_marker.scale = Vector3(
            x=self.floor_dims_[0],
            y=self.floor_dims_[1],
            z=self.floor_dims_[2]
        )
        floor_marker.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.8)
        floor_marker.lifetime = Duration(seconds=0).to_msg()
        self.floor_marker_pub_.publish(floor_marker)

        co = CollisionObject()
        co.header.frame_id = self.parent_frame_
        co.header.stamp = stamp
        co.id = "floor"
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            self.floor_dims_[0],
            self.floor_dims_[1],
            self.floor_dims_[2]
        ]
        co.primitives.append(primitive)
        co.primitive_poses.append(self.floor_pose_)
        self.collision_pub_.publish(co)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
