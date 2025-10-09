// this is a simple talker node that publishes strings to a topic
// currently it is not used in the project and it onle serves as a placeholder

#include <rclcpp/rclcpp.hpp> // needed for basic functions
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

class CollisionObjectPublisher : public rclcpp::Node
{
public:
  CollisionObjectPublisher() : Node("collision_object_publisher")
  {
    // Initialize PlanningSceneInterface
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Create and publish collision object
    publishCollisionObject();
  }

private:
  void publishCollisionObject()
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world"; // Set the frame of reference
    collision_object.id = "my_box"; // Unique ID for the object

    // Define the shape of the collision object (e.g., a box)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.5;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the collision object
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD; // Add the object to the scene

    // Apply the collision object to the planning scene
    planning_scene_interface_->applyCollisionObject(collision_object);
    RCLCPP_INFO(this->get_logger(), "Published collision object: %s", collision_object.id.c_str());
  }

  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionObjectPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}