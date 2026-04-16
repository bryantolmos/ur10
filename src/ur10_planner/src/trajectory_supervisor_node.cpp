#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include "ur10_planner/action/weld.hpp"
#include <mutex>      // Required for Point 1
#include <exception>  // Required for Point 2

class TrajectorySupervisorNode : public rclcpp::Node {
public:
    using WeldAction = ur10_planner::action::Weld;
    using GoalHandleWeld = rclcpp_action::ServerGoalHandle<WeldAction>;

    TrajectorySupervisorNode(const rclcpp::NodeOptions& options) 
    : Node("trajectory_supervisor", options) {
        
        if (!this->has_parameter("enable_rviz")) this->declare_parameter<bool>("enable_rviz", false);
        if (!this->has_parameter("use_sim_time")) this->declare_parameter<bool>("use_sim_time", true);
        if (!this->has_parameter("planning_group")) this->declare_parameter<std::string>("planning_group", "arm");
        if (!this->has_parameter("ee_link_name")) this->declare_parameter<std::string>("ee_link_name", "custom_tcp_link");
        if (!this->has_parameter("velocity_scaling")) this->declare_parameter<double>("velocity_scaling", 0.1);
        if (!this->has_parameter("acceleration_scaling")) this->declare_parameter<double>("acceleration_scaling", 0.1);

        traj_pub_ = this->create_publisher<moveit_msgs::msg::RobotTrajectory>("/planned_trajectory", 10);
        
        path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/welding_path", rclcpp::QoS(1).transient_local(),
            std::bind(&TrajectorySupervisorNode::path_callback, this, std::placeholders::_1));

        weld_action_server_ = rclcpp_action::create_server<WeldAction>(
            this,
            "execute_weld",
            std::bind(&TrajectorySupervisorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectorySupervisorNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&TrajectorySupervisorNode::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Trajectory Supervisor Action Server Ready (Bulletproofed).");
    }

private:
    std::mutex path_mutex_; // POINT 1: The memory lock
    geometry_msgs::msg::PoseArray::SharedPtr cached_path_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    void path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        // Lock the memory while writing to prevent race conditions
        std::lock_guard<std::mutex> lock(path_mutex_);
        cached_path_ = msg;
        RCLCPP_INFO(this->get_logger(), "Path cached securely with %zu waypoints.", msg->poses.size());
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const WeldAction::Goal>) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (!cached_path_ || cached_path_->poses.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Rejecting Action: No path in memory.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWeld>) {
        RCLCPP_WARN(this->get_logger(), "EMERGENCY ABORT: Stopping Robot!");
        if (move_group_) move_group_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleWeld> goal_handle) {
        std::thread{std::bind(&TrajectorySupervisorNode::execute_weld_routine, this, goal_handle)}.detach();
    }

    void execute_weld_routine(const std::shared_ptr<GoalHandleWeld> goal_handle) {
        auto result = std::make_shared<WeldAction::Result>();
        auto feedback = std::make_shared<WeldAction::Feedback>();

        // POINT 1: Deep copy the path so we don't hold the lock during the 30-second weld
        geometry_msgs::msg::PoseArray local_path;
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            local_path = *cached_path_;
        }

        if (!move_group_) {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), this->get_parameter("planning_group").as_string());
            move_group_->setEndEffectorLink(this->get_parameter("ee_link_name").as_string());
            move_group_->setMaxVelocityScalingFactor(this->get_parameter("velocity_scaling").as_double());
            move_group_->setMaxAccelerationScalingFactor(this->get_parameter("acceleration_scaling").as_double());
        }

        feedback->phase = "Phase 1: Approaching start position";
        goal_handle->publish_feedback(feedback);
        move_group_->setPlannerId("PTP");
        move_group_->setPoseTarget(local_path.poses[0]);

        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        if (move_group_->plan(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = false; result->message = "Approach Planning Failed.";
            goal_handle->abort(result); return;
        }
        move_group_->execute(approach_plan);

        feedback->phase = "Phase 2: Computing Cartesian Weave";
        goal_handle->publish_feedback(feedback);
        moveit_msgs::msg::RobotTrajectory raw_trajectory;
        double fraction = move_group_->computeCartesianPath(local_path.poses, 0.005, raw_trajectory);
        
        if (fraction < 0.95) {
            result->success = false; result->message = "Cartesian path unachievable.";
            goal_handle->abort(result); return;
        }

        feedback->phase = "Phase 3: Smoothing and Executing";
        goal_handle->publish_feedback(feedback);
        
        // POINT 2: Try-Catch block to prevent TOTG math from crashing the entire system
        moveit_msgs::msg::RobotTrajectory smoothed_trajectory;
        try {
            robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), "arm");
            moveit::core::RobotState reference_state(move_group_->getRobotModel());
            reference_state.setToDefaultValues();
            rt.setRobotTrajectoryMsg(reference_state, raw_trajectory);

            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            bool success = totg.computeTimeStamps(rt, 
                this->get_parameter("velocity_scaling").as_double(), 
                this->get_parameter("acceleration_scaling").as_double());

            if (!success) throw std::runtime_error("TOTG timestamp computation returned false.");

            rt.getRobotTrajectoryMsg(smoothed_trajectory);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Math Exception Caught: %s", e.what());
            result->success = false; 
            result->message = std::string("TOTG Math Failure: ") + e.what();
            goal_handle->abort(result);
            return;
        }

        traj_pub_->publish(smoothed_trajectory);

        if (move_group_->execute(smoothed_trajectory) == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true; result->message = "Weld Completed Flawlessly.";
            goal_handle->succeed(result);
        } else {
            result->success = false; result->message = "Hardware Execution Failed.";
            goal_handle->abort(result);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr traj_pub_;
    rclcpp_action::Server<WeldAction>::SharedPtr weld_action_server_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<TrajectorySupervisorNode>(options));
    rclcpp::shutdown();
    return 0;
}