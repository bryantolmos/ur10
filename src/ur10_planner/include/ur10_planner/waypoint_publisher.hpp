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
    explicit WaypointPublisher(const rclcpp::NodeOptions & options,
                               const std::string & node_name = "waypoint_publisher");

    // External trigger to start publishing (can be called from lifecycle node or main()).
    void start_timer();

private:
    // Core logic
    void publish_waypoints();
    bool load_parameters();
    void add_weave_segment(
        geometry_msgs::msg::PoseArray & msg,
        const geometry_msgs::msg::Pose & pose_a,
        const geometry_msgs::msg::Pose & pose_b);

    // Debug helper from Jesus' side
    void print_pose_array(const geometry_msgs::msg::PoseArray & pose_array);

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Path data
    std::vector<geometry_msgs::msg::Pose> main_path_waypoints_;

    // Parameters
    double weave_amplitude_ = 0.0;
    double weave_step_length_ = 0.01;

    std::string frame_id_{"base_link"};
    double default_roll_  = 0.0;  // radians
    double default_pitch_ = 0.0;  // radians
    double default_yaw_   = 0.0;  // radians
};

}

#endif