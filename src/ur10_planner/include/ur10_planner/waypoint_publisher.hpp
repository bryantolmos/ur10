#ifndef UR10_PLANNER__WAYPOINT_PUBLISHER_HPP_
#define UR10_PLANNER__WAYPOINT_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <chrono>

// It's good practice to wrap your package's classes in a namespace
namespace ur10_planner
{

class WaypointPublisher : public rclcpp::Node {
public:
    explicit WaypointPublisher(const rclcpp::NodeOptions & options);

private:
    void publish_waypoints();

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}

#endif