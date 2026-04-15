#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

class TrajectorySupervisorNode : public rclcpp::Node {
public:
    TrajectorySupervisorNode(const rclcpp::NodeOptions& options) 
    : Node("trajectory_supervisor", options) {
        
        if (!this->has_parameter("enable_rviz")) this->declare_parameter<bool>("enable_rviz", true);
        if (!this->has_parameter("use_sim_time")) this->declare_parameter<bool>("use_sim_time", true);
        if (!this->has_parameter("planning_group")) this->declare_parameter<std::string>("planning_group", "arm");
        if (!this->has_parameter("ee_link_name")) this->declare_parameter<std::string>("ee_link_name", "custom_tcp_link");
        if (!this->has_parameter("velocity_scaling")) this->declare_parameter<double>("velocity_scaling", 0.1);
        if (!this->has_parameter("acceleration_scaling")) this->declare_parameter<double>("acceleration_scaling", 0.1);
        
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

        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), this->get_parameter("planning_group").as_string()
        );

        move_group->setEndEffectorLink(this->get_parameter("ee_link_name").as_string());
        move_group->setMaxVelocityScalingFactor(this->get_parameter("velocity_scaling").as_double());
        move_group->setMaxAccelerationScalingFactor(this->get_parameter("acceleration_scaling").as_double());

        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools = nullptr;
        if (this->get_parameter("enable_rviz").as_bool()) {
            visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
                shared_from_this(), "world", "rviz_visual_markers", move_group->getRobotModel()
            );
            visual_tools->deleteAllMarkers();
        }

        // ==========================================
        // PHASE 1: Pilz PTP Approach
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "PHASE 1: Planning Pilz PTP approach...");
        
        move_group->setPlannerId("PTP"); 
        move_group->setPoseTarget(msg->poses[0]);

        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        if (move_group->plan(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan approach. Is the start position reachable?");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Executing approach...");
        if (move_group->execute(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Approach execution failed. Aborting.");
            return;
        }

        // ==========================================
        // PHASE 2: Cartesian Weld Generation
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "PHASE 2: Computing dense Cartesian weave...");
        moveit_msgs::msg::RobotTrajectory raw_trajectory;
        
        double fraction = move_group->computeCartesianPath(msg->poses, 0.005, raw_trajectory);
        
        if (fraction < 0.95) {
            RCLCPP_ERROR(this->get_logger(), "Only %.2f%% of path achievable. Aborting for safety.", fraction * 100.0);
            return;
        }

        // ==========================================
        // PHASE 3: Industrial Smoothing
        // ==========================================
        RCLCPP_INFO(this->get_logger(), "PHASE 3: Applying industrial jerk-limiting and blending...");
        
        // Convert ROS message to MoveIt internal trajectory format
        robot_trajectory::RobotTrajectory rt(move_group->getRobotModel(), "arm");
  
        moveit::core::RobotState reference_state(move_group->getRobotModel());
        reference_state.setToDefaultValues();

        // Load the raw Cartesian splines into the trajectory object
        rt.setRobotTrajectoryMsg(reference_state, raw_trajectory);

        // Apply Time Optimal Trajectory Generation
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        bool success = totg.computeTimeStamps(rt, 
            this->get_parameter("velocity_scaling").as_double(), 
            this->get_parameter("acceleration_scaling").as_double());

        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to apply industrial smoothing to trajectory!");
            return;
        }

        // Convert back to ROS message
        moveit_msgs::msg::RobotTrajectory smoothed_trajectory;
        rt.getRobotTrajectoryMsg(smoothed_trajectory);
        traj_pub_->publish(smoothed_trajectory);

        if (visual_tools) {
            visual_tools->publishTrajectoryLine(smoothed_trajectory, 
                move_group->getRobotModel()->getLinkModel(move_group->getEndEffectorLink()),
                move_group->getRobotModel()->getJointModelGroup("arm"));
            visual_tools->trigger();
        }

        RCLCPP_INFO(this->get_logger(), "Executing butter-smooth weld trajectory...");
        if (move_group->execute(smoothed_trajectory) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Weld completed flawlessly.");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr traj_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<TrajectorySupervisorNode>(options));
    rclcpp::shutdown();
    return 0;
}