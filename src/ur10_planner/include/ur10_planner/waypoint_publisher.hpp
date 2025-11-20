#ifndef UR10_PLANNER__WAYPOINT_PUBLISHER_HPP_
#define UR10_PLANNER__WAYPOINT_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <string>
#include <vector>

namespace ur10_planner
{

class WaypointPublisher : public rclcpp::Node {
public:
    explicit WaypointPublisher(const rclcpp::NodeOptions & options, const std::string & node_name = "waypoint_publisher");
    void start_timer();

private:
    void publish_waypoints();
    bool load_parameters();
    void add_weave_segment(
        geometry_msgs::msg::PoseArray & msg, 
        const geometry_msgs::msg::Pose & pose_a, 
        const geometry_msgs::msg::Pose & pose_b);

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Pose> main_path_waypoints_;
    double weave_amplitude_ = 0.0;
    double weave_step_length_ = 0.01;
};

}

#endif