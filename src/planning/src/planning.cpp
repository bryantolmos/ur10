#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ur10_planning_node");
    RCLCPP_INFO(node->get_logger(), "Planning node has started.");

    // Create MoveGroupInterface for UR10
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator"); // Manipulator is the default group name for UR robots

    // Set a target pose for the end effector
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.position.x = 0.4; // 40 cm forward, just an example and for testing
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.4; 
    move_group_interface.setPoseTarget(target_pose);
 
    // Plan the motion to the target pose
    MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        RCLCPP_INFO(node->get_logger(), "Planning successful.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning failed.");
    }
    
    // Execute the planned trajectory
    if (success) {
        move_group_interface.execute(my_plan);
        RCLCPP_INFO(node->get_logger(), "Executing the planned trajectory.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Execution failed.");
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}