#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <lifecycle_msgs/msg/state.hpp> // FIX: Included the missing state definitions
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include "ur10_planner/action/weld.hpp"
#include <mutex>
#include <atomic>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TrajectorySupervisorNode : public rclcpp_lifecycle::LifecycleNode {
public:
    using WeldAction = ur10_planner::action::Weld;
    using GoalHandleWeld = rclcpp_action::ServerGoalHandle<WeldAction>;

    // FIX: Constructor now accepts the invisible standard node to hand to MoveIt
    TrajectorySupervisorNode(const rclcpp::NodeOptions& options, rclcpp::Node::SharedPtr moveit_node) 
    : rclcpp_lifecycle::LifecycleNode("trajectory_supervisor", options), moveit_node_(moveit_node) {
        RCLCPP_INFO(this->get_logger(), "Lifecycle Node Booted: Currently in UNCONFIGURED state.");
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
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

        approval_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/trajectory_approval", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                is_approved_ = msg->data;
                approval_received_ = true;
            });

        weld_action_server_ = rclcpp_action::create_server<WeldAction>(
            this, "execute_weld",
            std::bind(&TrajectorySupervisorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectorySupervisorNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&TrajectorySupervisorNode::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Configured successfully. Standing by in INACTIVE state.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        traj_pub_->on_activate();
        RCLCPP_INFO(this->get_logger(), "Node ACTIVATED. Ready to accept Action Goals.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        traj_pub_->on_deactivate();
        RCLCPP_WARN(this->get_logger(), "Node DEACTIVATED. Dropped to safe Standby mode.");
        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr moveit_node_; // The invisible standard node
    std::mutex path_mutex_;
    geometry_msgs::msg::PoseArray::SharedPtr cached_path_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp_lifecycle::LifecyclePublisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr approval_sub_;
    rclcpp_action::Server<WeldAction>::SharedPtr weld_action_server_;

    std::atomic<bool> approval_received_{false};
    std::atomic<bool> is_approved_{false};

    void path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        cached_path_ = msg;
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const WeldAction::Goal>) {
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_ERROR(this->get_logger(), "Rejecting Action: Node is currently in STANDBY mode.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::lock_guard<std::mutex> lock(path_mutex_);
        if (!cached_path_ || cached_path_->poses.empty()) return rclcpp_action::GoalResponse::REJECT;
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWeld>) {
        if (move_group_) move_group_->stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleWeld> goal_handle) {
        std::thread{std::bind(&TrajectorySupervisorNode::execute_weld_routine, this, goal_handle)}.detach();
    }

    void execute_weld_routine(const std::shared_ptr<GoalHandleWeld> goal_handle) {
        auto result = std::make_shared<WeldAction::Result>();
        auto feedback = std::make_shared<WeldAction::Feedback>();

        geometry_msgs::msg::PoseArray local_path;
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            local_path = *cached_path_;
        }

        if (!move_group_) {
            // FIX: Pass the standard moveit_node_ instead of the incompatible shared_from_this()
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                moveit_node_, this->get_parameter("planning_group").as_string());
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
        if (move_group_->computeCartesianPath(local_path.poses, 0.005, raw_trajectory) < 0.95) {
            result->success = false; result->message = "Cartesian path unachievable.";
            goal_handle->abort(result); return;
        }

        feedback->phase = "Phase 3: Smoothing Trajectory";
        goal_handle->publish_feedback(feedback);
        moveit_msgs::msg::RobotTrajectory smoothed_trajectory;
        try {
            robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), "arm");
            moveit::core::RobotState ref_state(move_group_->getRobotModel());
            ref_state.setToDefaultValues();
            rt.setRobotTrajectoryMsg(ref_state, raw_trajectory);

            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            if (!totg.computeTimeStamps(rt, 0.1, 0.1)) throw std::runtime_error("TOTG Failed.");
            rt.getRobotTrajectoryMsg(smoothed_trajectory);
        } catch (const std::exception& e) {
            result->success = false; result->message = "Math Exception Caught.";
            goal_handle->abort(result); return;
        }

        feedback->phase = "Phase 4: Awaiting Safety Validation";
        goal_handle->publish_feedback(feedback);
        
        approval_received_ = false;
        traj_pub_->publish(smoothed_trajectory);

        int wait_ms = 0;
        while (!approval_received_ && wait_ms < 5000) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            wait_ms += 100;
        }

        if (!approval_received_ || !is_approved_) {
            result->success = false; 
            result->message = (!approval_received_) ? "Safety Timeout!" : "EMERGENCY: Watchdog Rejected Path!";
            goal_handle->abort(result);
            return;
        }

        feedback->phase = "Phase 5: Executing Hardware Weld";
        goal_handle->publish_feedback(feedback);
        if (move_group_->execute(smoothed_trajectory) == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true; result->message = "Weld Completed Flawlessly.";
            goal_handle->succeed(result);
        } else {
            result->success = false; result->message = "Execution Failed.";
            goal_handle->abort(result);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    
    // FIX: Clone the configuration parameters into a fresh options struct for our invisible helper node
    rclcpp::NodeOptions helper_options;
    for (const auto& param : options.parameter_overrides()) {
        helper_options.append_parameter_override(param.get_name(), param.get_parameter_value());
    }
    helper_options.automatically_declare_parameters_from_overrides(true);

    // Create the invisible standard node for MoveIt
    auto moveit_helper = std::make_shared<rclcpp::Node>("trajectory_supervisor_moveit", helper_options);

    // Create the main Lifecycle Supervisor
    auto supervisor = std::make_shared<TrajectorySupervisorNode>(options, moveit_helper);

    // Spin both nodes simultaneously using a MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(supervisor->get_node_base_interface());
    executor.add_node(moveit_helper->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}