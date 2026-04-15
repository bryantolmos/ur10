#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

class SafetyValidatorNode : public rclcpp::Node {
public: 
    SafetyValidatorNode() : Node("safety_validator_node") {
        sub_ = this->create_subscription<moveit_msgs::msg::RobotTrajectory>(
            "/planned_trajectory", 10,
            std::bind(&SafetyValidatorNode::trajectory_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Safety Validator online. Monitoring /planned_trajectory...");
    }

private:
    void trajectory_callback(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Validating trajectory with %zu points...", msg->joint_trajectory.points.size());
        
        bool valid = true;
        for (size_t i = 0; i < msg->joint_trajectory.points.size(); ++i) {
            const auto& point = msg->joint_trajectory.points[i];
            
            for (size_t j = 0; j < point.positions.size(); ++j) {
                // Heuristic limits: standard UR10 joint limits approach +/- 2*PI
                if (point.positions[j] < -6.28 || point.positions[j] > 6.28) { 
                    RCLCPP_WARN(this->get_logger(), "Unsafe joint limit detected on point %zu, joint %zu: %.2f", i, j, point.positions[j]);
                    valid = false;
                }
            }
        }

        if (valid) {
            RCLCPP_INFO(this->get_logger(), "Trajectory passed safety constraints.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Trajectory is UNSAFE. (Note: Interlock not yet wired to Supervisor)");
        }
    }

    rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyValidatorNode>());
    rclcpp::shutdown();
    return 0;
}