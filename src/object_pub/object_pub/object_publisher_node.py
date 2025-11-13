import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
# import quality of service policies for publishers
from rclpy.qos import QoSProfile, DurabilityPolicy

import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose, Vector3, Quaternion
from visualization_msgs.msg import Marker
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

# math library is implicitly used via geometry_msgs types, but good practice to keep if direct use is planned
import math


class ObjectPublisherNode(Node):
    """
    Publishes TF, Marker, and CollisionObject for a known object at a fixed pose,
    and adds a static floor collision object.
    """
    def __init__(self):
        super().__init__('object_publisher_node')

        # declare node parameters
        # parameters for the target object
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('object_frame', 'known_object_frame')
        self.declare_parameter('pose.position.x', 0.5)
        self.declare_parameter('pose.position.y', 0.0)
        self.declare_parameter('pose.position.z', 0.03)
        self.declare_parameter('pose.orientation.x', 0.0)
        self.declare_parameter('pose.orientation.y', 0.0)
        self.declare_parameter('pose.orientation.z', 0.0)
        self.declare_parameter('pose.orientation.w', 1.0)
        self.declare_parameter('dimensions', [0.1, 0.5, 0.36]) # dimensions order: length, height, width
        self.declare_parameter('collision_padding', 0.0)
        self.declare_parameter('publish_rate_hz', 1.0)

        # parameters for the floor collision object
        self.declare_parameter('floor.enable', True)
        self.declare_parameter('floor.pose.position.x', 0.0)
        self.declare_parameter('floor.pose.position.y', 0.0)
        self.declare_parameter('floor.pose.position.z', -0.02) # place floor slightly below parent frame origin
        self.declare_parameter('floor.pose.orientation.w', 1.0) # assume flat orientation
        self.declare_parameter('floor.size', [2.0, 2.0, 0.01]) # floor dimensions (x, y, z)

        # retrieve parameter values
        self.parent_frame_ = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.object_frame_ = self.get_parameter('object_frame').get_parameter_value().string_value
        self.object_dims_ = self.get_parameter('dimensions').get_parameter_value().double_array_value
        self.collision_padding_ = self.get_parameter('collision_padding').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.object_pose_ = Pose()
        self.object_pose_.position.x = self.get_parameter('pose.position.x').get_parameter_value().double_value
        self.object_pose_.position.y = self.get_parameter('pose.position.y').get_parameter_value().double_value
        self.object_pose_.position.z = self.get_parameter('pose.position.z').get_parameter_value().double_value
        self.object_pose_.orientation.x = self.get_parameter('pose.orientation.x').get_parameter_value().double_value
        self.object_pose_.orientation.y = self.get_parameter('pose.orientation.y').get_parameter_value().double_value
        self.object_pose_.orientation.z = self.get_parameter('pose.orientation.z').get_parameter_value().double_value
        self.object_pose_.orientation.w = self.get_parameter('pose.orientation.w').get_parameter_value().double_value

        # retrieve floor parameters
        self.publish_floor_ = self.get_parameter('floor.enable').get_parameter_value().bool_value
        self.floor_pose_ = Pose()
        self.floor_pose_.position.x = self.get_parameter('floor.pose.position.x').get_parameter_value().double_value
        self.floor_pose_.position.y = self.get_parameter('floor.pose.position.y').get_parameter_value().double_value
        self.floor_pose_.position.z = self.get_parameter('floor.pose.position.z').get_parameter_value().double_value
        self.floor_pose_.orientation.x = 0.0 # floor orientation is kept simple (no roll/pitch)
        self.floor_pose_.orientation.y = 0.0
        self.floor_pose_.orientation.z = 0.0
        self.floor_pose_.orientation.w = self.get_parameter('floor.pose.orientation.w').get_parameter_value().double_value
        self.floor_dims_ = self.get_parameter('floor.size').get_parameter_value().double_array_value


        self.get_logger().info(f"Known object configured in frame '{self.object_frame_}' relative to '{self.parent_frame_}'")
        # log object configuration details if needed (pose/dims)
        if self.publish_floor_:
            self.get_logger().info(f"Floor collision object enabled at Z={self.floor_pose_.position.z:.3f} relative to '{self.parent_frame_}'")

        # setup static transform broadcaster
        self.tf_static_broadcaster_ = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_tf() # publish the transform once on startup

        # setup publishers for marker and collision object
        # use transient local qos to ensure messages are latched for late subscribers
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub_ = self.create_publisher(Marker, '/known_object_marker', latching_qos)
        # collision object publisher is used for both target and floor
        self.collision_pub_ = self.create_publisher(CollisionObject, '/collision_object', latching_qos)
        # separate publisher for the floor marker for debugging
        self.floor_marker_pub_ = self.create_publisher(Marker, '/floor_marker', latching_qos)

        # setup timer for periodic publishing or publish once
        if publish_rate > 0:
             # create a timer to call publish_all_objects periodically
             self.timer_ = self.create_timer(1.0 / publish_rate, self.publish_all_objects)
             self.get_logger().info(f"Publishing visuals periodically at {publish_rate} Hz.")
        else:
             self.publish_all_objects() # publish visuals only one time if rate is zero
             self.get_logger().info("Published visuals once.")


    def publish_static_tf(self):
        """Publishes the static transform for the known object."""
        t = TransformStamped()
        # populate the transform message fields
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
        # send the static transform
        self.tf_static_broadcaster_.sendTransform(t)
        self.get_logger().info(f"Published static TF: {self.parent_frame_} -> {self.object_frame_}")

    def publish_all_objects(self):
        """Publishes the Marker and CollisionObject messages for known objects."""
        now_msg = self.get_clock().now().to_msg()
        self.publish_target_object(now_msg)
        if self.publish_floor_:
            self.publish_floor_object(now_msg)
        # avoid excessive logging when using a timer
        # self.get_logger().debug("Published objects.")

    def publish_target_object(self, stamp):
        """Publishes Marker and CollisionObject for the target object."""
        # create and configure the marker message
        marker = Marker()
        marker.header.frame_id = self.parent_frame_
        marker.header.stamp = stamp
        marker.ns = "known_object"
        marker.id = 0 # unique id for the target object marker
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = self.object_pose_
        marker.scale = Vector3(x=self.object_dims_[0], y=self.object_dims_[1], z=self.object_dims_[2])
        marker.color = ColorRGBA(r=0.3, g=0.3, b=0.8, a=0.7) # set marker color (bluish)
        marker.lifetime = Duration(seconds=0).to_msg() # persistent marker
        self.marker_pub_.publish(marker)

        # create and configure the collision object message
        co = CollisionObject()
        co.header.frame_id = self.parent_frame_
        co.header.stamp = stamp
        co.id = "known_target_object" # unique id for the target collision object
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        # apply padding to the collision object dimensions
        primitive.dimensions = [
            self.object_dims_[0] + 2 * self.collision_padding_,
            self.object_dims_[1] + 2 * self.collision_padding_,
            self.object_dims_[2] + 2 * self.collision_padding_
        ]
        co.primitives.append(primitive)
        co.primitive_poses.append(self.object_pose_)
        self.collision_pub_.publish(co)

    def publish_floor_object(self, stamp):
        """Publishes Marker and CollisionObject for the floor."""
        # Publish Marker for the floor (for visualization)
        floor_marker = Marker()
        floor_marker.header.frame_id = self.parent_frame_
        floor_marker.header.stamp = stamp
        floor_marker.ns = "floor"
        floor_marker.id = 0
        floor_marker.type = Marker.CUBE
        floor_marker.action = Marker.ADD
        floor_marker.pose = self.floor_pose_
        floor_marker.scale = Vector3(x=self.floor_dims_[0], y=self.floor_dims_[1], z=self.floor_dims_[2])
        floor_marker.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.8)
        floor_marker.lifetime = Duration(seconds=0).to_msg()
        self.floor_marker_pub_.publish(floor_marker)

        # Publish CollisionObject for the floor (for MoveIt)
        co = CollisionObject()
        # floor pose is defined relative to the parent frame
        co.header.frame_id = self.parent_frame_
        co.header.stamp = stamp
        co.id = "floor" # unique id for the floor collision object
        co.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        # use floor dimensions directly, padding usually not needed for static environment
        primitive.dimensions = [
            self.floor_dims_[0],
            self.floor_dims_[1],
            self.floor_dims_[2]
        ]
        co.primitives.append(primitive)
        # apply the configured pose to the floor primitive
        co.primitive_poses.append(self.floor_pose_)
        self.collision_pub_.publish(co)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # handle ctrl+c cleanly
        pass
    finally:
        # ensure node is destroyed and rclpy is shut down
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# standard python entry point
if __name__ == '__main__':
    main()