#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <algorithm>  // Algorithm used for std::abs

class SafetyCheckerNode : public rclcpp::Node
{
public:
  SafetyCheckerNode()
  : Node("safety_checker_node")
  {
    RCLCPP_INFO(get_logger(), "Safety Checker Node initializing...");

    // --- Declare parameters ---
    declare_parameter<double>("max_joint_velocity", 1.5);       // [rad/s]
    declare_parameter<double>("max_joint_acceleration", 3.0);   // [rad/s^2]
    declare_parameter<bool>("enable_velocity_check", true);
    declare_parameter<bool>("enable_acceleration_check", true);

    // --- Get initial parameter values ---
    max_joint_velocity_ = get_parameter("max_joint_velocity").as_double();
    max_joint_acceleration_ = get_parameter("max_joint_acceleration").as_double();
    enable_velocity_check_ = get_parameter("enable_velocity_check").as_bool();
    enable_acceleration_check_ = get_parameter("enable_acceleration_check").as_bool();

    // --- Publisher ---
    safe_traj_pub_ = create_publisher<moveit_msgs::msg::RobotTrajectory>(
      "/safe_trajectory", rclcpp::QoS(rclcpp::KeepLast(10)));

    // --- Subscriber ---
    planned_traj_sub_ = create_subscription<moveit_msgs::msg::RobotTrajectory>(
      "/planned_trajectory",
      rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&SafetyCheckerNode::validate_plan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Safety Checker ready. Params -> max_vel: %.2f rad/s, max_acc: %.2f rad/s²",
                max_joint_velocity_, max_joint_acceleration_);
  }

private:
  void validate_plan_callback(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received a planned trajectory for validation.");

    // --- Check 1: Empty trajectory ---
    if (msg->joint_trajectory.points.empty())
    {
      RCLCPP_ERROR(get_logger(), "Validation failed: trajectory has no points.");
      return;
    }

    // --- Check 2: Velocity and acceleration thresholds ---
    if (!perform_safety_checks(msg))
    {
      RCLCPP_ERROR(get_logger(), "Validation failed: trajectory violates safety limits.");
      return;
    }

    // --- Success: publish ---
    RCLCPP_INFO(get_logger(), "Plan validated successfully. Publishing to /safe_trajectory.");
    safe_traj_pub_->publish(*msg);
  }

  bool perform_safety_checks(const moveit_msgs::msg::RobotTrajectory::SharedPtr &msg)
  {
    const auto &points = msg->joint_trajectory.points;
    const size_t num_points = points.size();

    for (size_t i = 0; i < num_points; ++i)
    {
      // Velocity check
      if (enable_velocity_check_)
      {
        for (const auto &vel : points[i].velocities)
        {
          if (std::abs(vel) > max_joint_velocity_)
          {
            RCLCPP_ERROR(get_logger(),
                         "Velocity limit exceeded at point %zu: %.3f > %.3f rad/s",
                         i, vel, max_joint_velocity_);
            return false;
          }
        }
      }

      // Acceleration check (if available)
      if (enable_acceleration_check_ && !points[i].accelerations.empty())
      {
        for (const auto &acc : points[i].accelerations)
        {
          if (std::abs(acc) > max_joint_acceleration_)
          {
            RCLCPP_ERROR(get_logger(),
                         "Acceleration limit exceeded at point %zu: %.3f > %.3f rad/s²",
                         i, acc, max_joint_acceleration_);
            return false;
          }
        }
      }
    }

    return true;  // All checks passed
  }

  // ROS Interfaces
  rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr planned_traj_sub_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr safe_traj_pub_;

  // Parameters
  double max_joint_velocity_; //Measured in rad/s; maximum allowed value for joint velocity
  double max_joint_acceleration_; //Measured in rad/s^2; maximum allowed value for joint acceleration
  bool enable_velocity_check_; //Toggling safety checks for velocity + acceleration (see next parameter)
  bool enable_acceleration_check_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyCheckerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// Launch with "ros2 run ur10_planner safety_checker_node --ros-args --params-file config/safety_checker_params.yaml"