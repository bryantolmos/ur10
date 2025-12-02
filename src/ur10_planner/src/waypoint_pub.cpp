#include "../include/ur10_planner/waypoint_publisher.hpp"
#include <chrono>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ur10_planner {

WaypointPublisher::WaypointPublisher(const rclcpp::NodeOptions &options, const std::string & node_name)
    : Node(node_name, options), timer_(nullptr)
{
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/welding_path", 10);
    
    // declare parameters
    this->declare_parameter("weave_amplitude", 0.05);
    this->declare_parameter("weave_step_length", 0.02);
    // declare FLATTENED path array
    this->declare_parameter("main_path_flat", std::vector<double>{});

    RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Node has started. Waiting for activation trigger.");
}

void WaypointPublisher::start_timer()
{
    RCLCPP_INFO(this->get_logger(), "Activating waypoint publisher timer");
    if(!timer_) {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&WaypointPublisher::publish_waypoints, this));
    } else {
        RCLCPP_WARN(this->get_logger(), "Timer already active.");
    }
}

bool WaypointPublisher::load_parameters()
{
    RCLCPP_INFO(this->get_logger(), "Loading path and weave parameters from YAML...");
    
    // get weave parameters
    this->get_parameter("weave_amplitude", weave_amplitude_);
    this->get_parameter("weave_step_length", weave_step_length_);
    
    if (weave_step_length_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "weave_step_length must be positive.");
        return false;
    }

    // get the flattened path parameter
    std::vector<double> path_param;
    this->get_parameter("main_path_flat", path_param);

    // check if its a multiple of 6 (x,y,z,r,p,y)
    if (path_param.empty() || path_param.size() % 6 != 0) {
        RCLCPP_ERROR(this->get_logger(), "'main_path_flat' has %zu elements, which is not a multiple of 6.", path_param.size());
        return false;
    }

    main_path_waypoints_.clear();
    tf2::Quaternion q;

    // reconstruct the pose vector from the flat array
    for (size_t i = 0; i < path_param.size(); i += 6) {
        geometry_msgs::msg::Pose p;
        p.position.x = path_param[i + 0];
        p.position.y = path_param[i + 1];
        p.position.z = path_param[i + 2];

        q.setRPY(path_param[i + 3], path_param[i + 4], path_param[i + 5]);
        p.orientation = tf2::toMsg(q);
        main_path_waypoints_.push_back(p);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu main waypoints.", main_path_waypoints_.size());
    return true;
}


void WaypointPublisher::add_weave_segment(
    geometry_msgs::msg::PoseArray & msg, 
    const geometry_msgs::msg::Pose & pose_a, 
    const geometry_msgs::msg::Pose & pose_b)
{
    // convert poses to tf2 objects for easy math
    tf2::Transform tf_a, tf_b;
    tf2::fromMsg(pose_a, tf_a);
    tf2::fromMsg(pose_b, tf_b);

    // get path vector
    tf2::Vector3 path_vector = tf_b.getOrigin() - tf_a.getOrigin();
    double path_distance = path_vector.length();

    if (path_distance < 1e-6) {
        msg.poses.push_back(pose_a);
        return;
    }

    tf2::Vector3 path_direction = path_vector.normalized();

    // get waeve direction (perpendicular to path)
    tf2::Vector3 world_up(0.0, 0.0, 1.0);
    tf2::Vector3 weave_direction = path_direction.cross(world_up);
    
    if (weave_direction.length() < 1e-6) {
        weave_direction = tf2::Vector3(1.0, 0.0, 0.0);
    }
    weave_direction.normalize();

    // generate the interpolated points
    int num_steps = static_cast<int>(path_distance / weave_step_length_);
    if (num_steps < 2) num_steps = 2; // ensure at least start and end points

    for (int i = 0; i < num_steps; ++i) {
        double t = static_cast<double>(i) / (num_steps - 1);

        // interpolate orientation using slerp
        tf2::Quaternion interp_q = tf_a.getRotation().slerp(tf_b.getRotation(), t);

        // interpolate position
        tf2::Vector3 interp_pos = tf_a.getOrigin().lerp(tf_b.getOrigin(), t);

        double weave_offset = (i % 2 == 1) ? weave_amplitude_ : -weave_amplitude_;
        if (i == 0 || i == (num_steps - 1)) {
            weave_offset = 0.0;
        }

        tf2::Vector3 final_pos = interp_pos + (weave_direction * weave_offset);

        geometry_msgs::msg::Pose p;
        
        p.position.x = final_pos.x();
        p.position.y = final_pos.y();
        p.position.z = final_pos.z();
        
        p.orientation = tf2::toMsg(interp_q);
        msg.poses.push_back(p);
    }
}


void WaypointPublisher::publish_waypoints()
{
    if(timer_) {
        timer_->cancel();
    }

    if (!load_parameters()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load parameters. Path not published.");
        return;
    }

    if (main_path_waypoints_.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Main path needs at least 2 waypoints. Path not published.");
        return;
    }

    auto message = geometry_msgs::msg::PoseArray();
    message.header.stamp = this->now();
    message.header.frame_id = "world";

    for (size_t i = 0; i < main_path_waypoints_.size() - 1; ++i) {
        const auto& pose_a = main_path_waypoints_[i];
        const auto& pose_b = main_path_waypoints_[i+1];
        
        RCLCPP_INFO(this->get_logger(), "Generating weave segment %zu...", i);
        add_weave_segment(message, pose_a, pose_b);
    }

    // add the very last waypoint.
    if (!main_path_waypoints_.empty()) {
        message.poses.push_back(main_path_waypoints_.back());
    }

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing PoseArray with %zu total waypoints (full weave path).", message.poses.size());
}

}