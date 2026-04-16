#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>

class SafetyValidatorNode : public rclcpp::Node {
public:
    SafetyValidatorNode(const rclcpp::NodeOptions& options) 
    : Node("safety_validator", options) {
        
        sub_ = this->create_subscription<moveit_msgs::msg::RobotTrajectory>(
            "/planned_trajectory", 10,
            std::bind(&SafetyValidatorNode::trajectory_callback, this, std::placeholders::_1));
            
        pub_ = this->create_publisher<std_msgs::msg::Bool>("/trajectory_approval", 10);
        RCLCPP_INFO(this->get_logger(), "Safety Validator ARMED. Acting as Gatekeeper.");
    }

private:
    void trajectory_callback(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg) {
        bool is_safe = true;

        // Strict Check: Ensure no joint exceeds UR10 physical limits (-2π to 2π)
        for (const auto& point : msg->joint_trajectory.points) {
            for (double pos : point.positions) {
                if (pos < -6.28 || pos > 6.28) is_safe = false;
            }
        }

        std_msgs::msg::Bool approval_msg;
        approval_msg.data = is_safe;
        pub_->publish(approval_msg);

        if (is_safe) {
            RCLCPP_INFO(this->get_logger(), "Trajectory VALIDATED. Passing control back to Supervisor.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY: Trajectory violates joint limits! REJECTED.");
        }
    }

    rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<SafetyValidatorNode>(options));
    rclcpp::shutdown();
    return 0;
}