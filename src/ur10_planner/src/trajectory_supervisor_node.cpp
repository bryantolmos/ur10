#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

class TrajectorySupervisorNode : public rclcpp::Node {
public:
    TrajectorySupervisorNode(const rclcpp::NodeOptions& options) 
    : Node("trajectory_supervisor", options) {
        
        if (!this->has_parameter("enable_rviz")) {
            this->declare_parameter<bool>("enable_rviz", false);
        }
        
        traj_pub_ = this->create_publisher<moveit_msgs::msg::RobotTrajectory>("/planned_trajectory", 10);
        path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/welding_path", rclcpp::QoS(1).transient_local(),
            std::bind(&TrajectorySupervisorNode::path_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Trajectory Supervisor online. Waiting for path on /welding_path...");
    }

private:
    void path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Path received. Initiating MoveIt 2...");

        moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "arm");
        move_group.setEndEffectorLink("custom_tcp_link"); 
        
        // Slow down for safety
        move_group.setMaxVelocityScalingFactor(0.1); 
        move_group.setMaxAccelerationScalingFactor(0.1);

        // ==========================================
        // PHASE 1: The Approach
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "PHASE 1: Planning approach to the start position...");
        move_group.setPoseTarget(msg->poses[0]);
        
        moveit::core::MoveItErrorCode approach_result = move_group.move();
        
        if (approach_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to approach the starting position. Aborting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Robot is in position. Beginning weld path...");

        // ==========================================
        // PHASE 2: The Cartesian Weld
        // ==========================================
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group.computeCartesianPath(msg->poses, 0.01, trajectory);

        RCLCPP_INFO(this->get_logger(), "PHASE 2: Cartesian path computation achieved %.2f%%", fraction * 100.0);

        if (fraction > 0.9) {
            traj_pub_->publish(trajectory);

            if (this->get_parameter("enable_rviz").as_bool()) {
                namespace rvt = rviz_visual_tools;
                moveit_visual_tools::MoveItVisualTools visual_tools(shared_from_this(), "world", "rviz_visual_markers", move_group.getRobotModel());
                
                const moveit::core::LinkModel* ee_link_model = move_group.getRobotModel()->getLinkModel(move_group.getEndEffectorLink());
                const moveit::core::JointModelGroup* jmg = move_group.getRobotModel()->getJointModelGroup("arm");
                
                visual_tools.publishTrajectoryLine(trajectory, ee_link_model, jmg);
                visual_tools.trigger();
            }

            RCLCPP_INFO(this->get_logger(), "Executing weld trajectory...");
            move_group.execute(trajectory);
            RCLCPP_INFO(this->get_logger(), "Weld Complete!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Path computation failed (likely due to joint limits or singularity).");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr traj_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<TrajectorySupervisorNode>(options);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}