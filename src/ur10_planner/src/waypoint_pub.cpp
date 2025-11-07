#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <chrono>

#include "../include/ur10_planner/waypoint_publisher.hpp"

namespace ur10_planner {
WaypointPublisher::WaypointPublisher(const rclcpp::NodeOptions &options)
    : Node("waypoint_publisher", options) // pass options to the base node class
{
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/welding_path", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&WaypointPublisher::publish_waypoints, this));

    RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Node has started.");
}

void WaypointPublisher::publish_waypoints()
{
    auto message = geometry_msgs::msg::PoseArray();
    message.header.stamp = this->now();
    message.header.frame_id = "world";

    // First waypoint
    geometry_msgs::msg::Pose pose1;
    pose1.position.x = -1.0;
    pose1.position.y = 0.5;
    pose1.position.z = 0.5;
    tf2::Quaternion q1;
    q1.setRPY(0, M_PI, 0);
    pose1.orientation.x = q1.x();
    pose1.orientation.y = q1.y();
    pose1.orientation.z = q1.z();
    pose1.orientation.w = q1.w();
    message.poses.push_back(pose1);

    // Second waypoint
    geometry_msgs::msg::Pose pose2;
    pose2.position.x = -1.0;
    pose2.position.y = -0.5;
    pose2.position.z = 0.5;
    tf2::Quaternion q2;
    q2.setRPY(0, 0, 0);
    pose2.orientation.x = q2.x();
    pose2.orientation.y = q2.y();
    pose2.orientation.z = q2.z();
    pose2.orientation.w = q2.w();
    message.poses.push_back(pose2);

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing PoseArray.");
}

}