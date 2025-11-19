#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <chrono>

class WaypointSubscriber : public rclcpp::Node {
public:
    WaypointSubscriber() : Node("WaypointSubscriber") {

        auto sub_qos = rclcpp::QoS(10);
        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                                .transient_local() // ensures last message is available to new subscribers
                                .reliable(); // ensures delivery, meaning the message is not lost

        // Create subscription to "/welding_path" from frontend
        sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/welding_path",
            sub_qos,
            std::bind(&WaypointSubscriber::waypoint_callback, this, std::placeholders::_1)
        );

        // Latched publisher to "/welding_path" for planner/executor to use
        pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/planned_trajectory",
            pub_qos
        );

        RCLCPP_INFO(this->get_logger(),
                    "WaypointRelayNode started. Listening to /welding_path, "
                    "latched publish on /planned_trajectory.");
    }

private:
    void waypoint_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses.", msg->poses.size());
        print_pose_array(*msg);

        if (!published_once_) {
            pub_->publish(*msg);
            published_once_ = true;
            RCLCPP_INFO(this->get_logger(), "PoseArray latched on /planned_trajectory."
                                            "Future subscribers will receive this path.");
        } else {
            RCLCPP_INFO(this->get_logger(), "PoseArray already latched, ignoring input.");
        }
    }

    void print_pose_array(const geometry_msgs::msg::PoseArray & pose_array) {
        RCLCPP_INFO(this->get_logger(), "PoseArray has %zu poses.", pose_array.poses.size());
        for (size_t i = 0; i < pose_array.poses.size(); ++i) {
            const auto & pose = pose_array.poses[i];
            RCLCPP_INFO(
                this->get_logger(),
                "Pose %zu: Position(%.2f, %.2f, %.2f), Orientation(%.2f, %.2f, %.2f, %.2f)",
                i,
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            );
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
    bool published_once_ = false;
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointSubscriber>());
    rclcpp::shutdown();
    return 0;
}