#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <chrono>

class WaypointPublisher : public rclcpp::Node {
public:
    WaypointPublisher() : Node("waypoint_sender") {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/welding_path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WaypointPublisher::send_waypoints, this)
        );

        RCLCPP_INFO(this->get_logger(), "Invalid Waypoint Sender Node has started.");
        // Additional initialization and publisher setup can be done here
    }

private:
    void send_waypoints() {
        auto message = geometry_msgs::msg::PoseArray();
        message.header.stamp = this->now(); 
        message.header.frame_id = "world";

        /* From github:
        - simple path (e.g., a 10cm line in the x-direction,
        with the end-effector pointing straight down).
        */
        
        // Invalid waypoint, below floor level 0
        geometry_msgs::msg::Pose pose1;
        pose1.position.x = -1.0;
        pose1.position.y = 0.5;
        pose1.position.z = -5.0;
        tf2::Quaternion q1;
        q1.setRPY(0, M_PI, 0); 
        pose1.orientation.x = q1.x();
        pose1.orientation.y = q1.y();
        pose1.orientation.z = q1.z();
        pose1.orientation.w = q1.w();
        message.poses.push_back(pose1);

        publisher_->publish(message);
        int count = message.poses.size();
        RCLCPP_INFO(this->get_logger(), "Publishing Invalid PoseArray with %d poses.", count);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto waypoint_sender_node = std::make_shared<WaypointPublisher>();
    rclcpp::spin(waypoint_sender_node);
    rclcpp::shutdown();
    return 0;
}