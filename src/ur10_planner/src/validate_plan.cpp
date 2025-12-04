#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <cmath>
#include <chrono>

class PathValidator : public rclcpp::Node {
public: 
    //PathValidator() : Node("PathValidator") {
    //    auto sub_ = this->create_subscription<moveit_msgs::msg::RobotTrajectory>(
    //        "/planned_trajectory",
    //        rclcpp::QoS(10),
    //        std::bind(&PathValidator::trajectory_callback, this, std::placeholders::_1)
    //    );
    //}

    PathValidator() : Node("PathValidator") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                        .transient_local() // ensures last message is available to new subscribers
                        .reliable(); // ensures delivery, meaning the message is not lost

        sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/planned_trajectory",
            qos,
            std::bind(&PathValidator::posearray_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "PathValidator node has been started.");
    }

private:

    /* Leaving this commented functions in case we do need them.
                            ( ͡° ͜ʖ ͡°)
    */

    //void trajectory_callback(const moveit_msgs::msg::RobotTrajectory::SharedPtr msg) {
    //    RCLCPP_INFO(this->get_logger(), "Received RobotTrajectory with %zu points.", msg->joint_trajectory.points.size());
    //    validate_trajectory(*msg);
    //}
    
    //void validate_trajectory(const moveit_msgs::msg::RobotTrajectory & trajectory) {
    //    bool valid = true;
    //    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
    //        const auto & point = trajectory.joint_trajectory.points[i];
    //        // Example validation: Check if positions are within some limits
    //        for (const auto & position : point.positions) {
    //            if (position < -3.14 || position > 3.14) { // Example joint limits
    //                RCLCPP_WARN(this->get_logger(), "Point %zu has invalid joint position: %.2f", i, position);
    //                valid = false;
    //            } else if (trajectory.joint_trajectory.points.size() < 1) {
    //                RCLCPP_WARN(this->get_logger(), "Trajectory has no points.");
    //                valid = false;
    //            }
    //        }
    //    }
    //    if (valid) {
    //        RCLCPP_INFO(this->get_logger(), "Trajectory is valid.");
    //    } else {
    //        RCLCPP_ERROR(this->get_logger(), "Trajectory validation failed.");
    //    }
    //}

    void posearray_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses.", msg->poses.size());
        validate_trajectory(*msg);
    }
    
    void validate_trajectory(const geometry_msgs::msg::PoseArray & trajectory) {
        bool valid = true;
        for (size_t i = 0; i < trajectory.poses.size(); ++i) {
            const auto & pose = trajectory.poses[i];
            if (trajectory.poses.size() < 1) {
                RCLCPP_WARN(this->get_logger(), "Trajectory has no poses.");
                valid = false;
            }
            // Example validation: Check if positions are within some limits
            if (trajectory.poses[i].position.z < 0.0) {
                RCLCPP_WARN(this->get_logger(), "Pose %zu is below floor level.", i);
                valid = false;
            }
            tf2::Quaternion q(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            );
            if (q.length2() < 1e-6) {
                RCLCPP_WARN(this->get_logger(), "Pose %zu has invalid orientation (zero length quaternion).", i);
                valid = false;
            }
        }
        if (valid) {
            RCLCPP_INFO(this->get_logger(), "Trajectory is valid.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Trajectory validation failed.");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathValidator>());
    rclcpp::shutdown();
    return 0;
}