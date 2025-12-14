#include "../include/ur10_planner/waypoint_publisher.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <cmath>
#include <fstream>

namespace ur10_planner
{

WaypointPublisher::WaypointPublisher(const rclcpp::NodeOptions & options,
                                     const std::string & node_name)
  : rclcpp::Node(node_name, options),
    timer_(nullptr) {
  // Weave parameters
  this->declare_parameter<double>("weave_amplitude", 0.05);
  this->declare_parameter<double>("weave_step_length", 0.02);

  // Frame & default orientation (placeholder – to be replaced with surface normal logic, right now just defaults)
  this->declare_parameter<std::string>("frame_id", "base_link");
  this->declare_parameter<double>("default_roll", 0.0);   // rad
  this->declare_parameter<double>("default_pitch", 0.0);  // rad
  this->declare_parameter<double>("default_yaw", 0.0);    // rad

  // Default to empty. If set, it overrides manual params.
  this->declare_parameter<std::string>("points_file_path", ""); 

  // Legacy params (keep them as fallback)
  this->declare_parameter<std::vector<double>>("main_path_flat", std::vector<double>{});
  this->declare_parameter<std::vector<double>>("selected_points", std::vector<double>{});

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/welding_path", qos);

  RCLCPP_INFO(this->get_logger(), "WaypointPublisher initialized.");
}

void WaypointPublisher::start_timer()
{
  RCLCPP_INFO(this->get_logger(), "Activating waypoint publisher timer");

  // allow re-triggering eg clicling 'plan' multiple times
  if (timer_) {
      timer_->cancel();
  }
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&WaypointPublisher::publish_waypoints, this));
}

// parse yaml
bool WaypointPublisher::load_from_yaml_file(const std::string& path) {
    std::ifstream fin(path);
    if (!fin.fail()) {
        RCLCPP_INFO(this->get_logger(), "Reading points from file: %s", path.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "File not found: %s", path.c_str());
        return false;
    }

    try {
        YAML::Node config = YAML::LoadFile(path);

        // Check if root node "selected_points" exists
        if (!config["selected_points"]) {
            RCLCPP_ERROR(this->get_logger(), "YAML file missing 'selected_points' key.");
            return false;
        }

        // Prepare default orientation
        tf2::Quaternion q_default;
        q_default.setRPY(default_roll_, default_pitch_, default_yaw_);
        geometry_msgs::msg::Quaternion q_msg_default = tf2::toMsg(q_default);

        // Clear existing
        main_path_waypoints_.clear();

        const YAML::Node& points = config["selected_points"];
        for (std::size_t i = 0; i < points.size(); ++i) {
            geometry_msgs::msg::Pose p;
            
            // Read x, y, z from the map
            p.position.x = points[i]["x"].as<double>();
            p.position.y = points[i]["y"].as<double>();
            p.position.z = points[i]["z"].as<double>();
            
            // Apply default orientation
            p.orientation = q_msg_default;

            main_path_waypoints_.push_back(p);
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu points from YAML.", main_path_waypoints_.size());
        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "YAML Parsing Error: %s", e.what());
        return false;
    }
}



// Parameter loading
bool WaypointPublisher::load_parameters() {
  RCLCPP_INFO(this->get_logger(), "Loading path and weave parameters...");

  // Weave parameters
  this->get_parameter("weave_amplitude", weave_amplitude_);
  this->get_parameter("weave_step_length", weave_step_length_);

  if (weave_step_length_ <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "weave_step_length must be positive.");
    return false;
  }


  // Frame and default orientation
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("default_roll", default_roll_);
  this->get_parameter("default_pitch", default_pitch_);
  this->get_parameter("default_yaw", default_yaw_);

  // get file path
  this->get_parameter("points_file_path", points_file_path_);

  if (!points_file_path_.empty()) {
    if (load_from_yaml_file(points_file_path_)) {
      if (main_path_waypoints_.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Path file had fewer than 2 points.");
        return false;
      }
      return true;
    }
  }

  // Input options
  std::vector<double> main_path_flat;
  std::vector<double> selected_points_flat;
  this->get_parameter("main_path_flat", main_path_flat);
  this->get_parameter("selected_points", selected_points_flat);

  main_path_waypoints_.clear();

  if (!main_path_flat.empty()) {
    // --- Option 1: Full pose + RPY per waypoint ---
    if (main_path_flat.size() % 6 != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "'main_path_flat' has %zu elements, which is not a multiple of 6.",
                   main_path_flat.size());
      return false;
    }

    tf2::Quaternion q;
    for (size_t i = 0; i < main_path_flat.size(); i += 6) {
      geometry_msgs::msg::Pose p;
      p.position.x = main_path_flat[i + 0];
      p.position.y = main_path_flat[i + 1];
      p.position.z = main_path_flat[i + 2];

      double roll  = main_path_flat[i + 3];
      double pitch = main_path_flat[i + 4];
      double yaw   = main_path_flat[i + 5];

      q.setRPY(roll, pitch, yaw);
      p.orientation = tf2::toMsg(q);
      main_path_waypoints_.push_back(p);
    }

    RCLCPP_INFO(this->get_logger(),
                "Loaded %zu main waypoints from 'main_path_flat'.",
                main_path_waypoints_.size());
  }
  else if (!selected_points_flat.empty()) {
    // --- Option 2: Positions only (from selected_points.yaml) ---
    if (selected_points_flat.size() % 3 != 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "'selected_points' has %zu elements, which is not a multiple of 3.",
                   selected_points_flat.size());
      return false;
    }

    // Build a quaternion from default RPY.
    tf2::Quaternion q_default;
    q_default.setRPY(default_roll_, default_pitch_, default_yaw_);
    geometry_msgs::msg::Quaternion q_msg_default = tf2::toMsg(q_default);

    for (size_t i = 0; i < selected_points_flat.size(); i += 3) {
      geometry_msgs::msg::Pose p;
      p.position.x = selected_points_flat[i + 0];
      p.position.y = selected_points_flat[i + 1];
      p.position.z = selected_points_flat[i + 2];

      // TODO: Replace this fixed orientation with logic that uses:
      //  - local surface normal
      //  - weld direction along the path
      // For now, use a default orientation from parameters so it's easy to swap later.
      p.orientation = q_msg_default;

      main_path_waypoints_.push_back(p);
    }

    RCLCPP_INFO(this->get_logger(),
                "Loaded %zu main waypoints from 'selected_points' (positions only).",
                main_path_waypoints_.size());
  }
  else {
    RCLCPP_ERROR(this->get_logger(),
                 "No input path provided. Set either 'main_path_flat' or 'selected_points'.");
    return false;
  }

  if (main_path_waypoints_.size() < 2) {
    RCLCPP_ERROR(this->get_logger(),
                 "Main path needs at least 2 waypoints (got %zu).",
                 main_path_waypoints_.size());
    return false;
  }

  return true;
}

// Weave segment generation
void WaypointPublisher::add_weave_segment(
    geometry_msgs::msg::PoseArray & msg,
    const geometry_msgs::msg::Pose & pose_a,
    const geometry_msgs::msg::Pose & pose_b)
{
  tf2::Transform tf_a, tf_b;
  tf2::fromMsg(pose_a, tf_a);
  tf2::fromMsg(pose_b, tf_b);

  tf2::Vector3 path_vector = tf_b.getOrigin() - tf_a.getOrigin();
  double path_distance = path_vector.length();

  if (path_distance < 1e-6) {
    // Degenerate segment – just push pose A
    msg.poses.push_back(pose_a);
    return;
  }

  tf2::Vector3 path_direction = path_vector.normalized();

  // Weave direction: perpendicular to path, using world-up as reference
  tf2::Vector3 world_up(0.0, 0.0, 1.0);
  tf2::Vector3 weave_direction = path_direction.cross(world_up);

  if (weave_direction.length() < 1e-6) {
    // If path is parallel to world_up, choose arbitrary axis
    weave_direction = tf2::Vector3(1.0, 0.0, 0.0);
  }
  weave_direction.normalize();

  int num_steps = static_cast<int>(path_distance / weave_step_length_);
  if (num_steps < 2) {
    num_steps = 2;  // always at least start & end
  }

  for (int i = 0; i < num_steps; ++i) {
    double t = static_cast<double>(i) / (num_steps - 1);

    // Interpolate orientation (slerp) and position (lerp)
    tf2::Quaternion interp_q =
      tf_a.getRotation().slerp(tf_b.getRotation(), t);
    tf2::Vector3 interp_pos =
      tf_a.getOrigin().lerp(tf_b.getOrigin(), t);

    // Alternating weave offset; zero at endpoints
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

// Publishing logic
void WaypointPublisher::publish_waypoints()
{
  // One-shot: cancel timer if it exists.
  if (timer_) {
    timer_->cancel();
  }

  if (!load_parameters()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to load parameters. Path not published.");
    return;
  }

  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;

  for (size_t i = 0; i < main_path_waypoints_.size() - 1; ++i) {
    const auto & pose_a = main_path_waypoints_[i];
    const auto & pose_b = main_path_waypoints_[i + 1];

    RCLCPP_INFO(this->get_logger(), "Generating weave segment %zu...", i);
    add_weave_segment(msg, pose_a, pose_b);
  }

  // Add final waypoint explicitly
  msg.poses.push_back(main_path_waypoints_.back());

  // Log full path for debugging
  print_pose_array(msg);

  // Publish on /welding_path
  publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
              "Published PoseArray with %zu total waypoints on /welding_path.",
              msg.poses.size());
}

// ===========================
// Debug logging helper
// ===========================
void WaypointPublisher::print_pose_array(const geometry_msgs::msg::PoseArray & pose_array)
{
  RCLCPP_INFO(this->get_logger(),
              "PoseArray has %zu poses (frame_id='%s').",
              pose_array.poses.size(), pose_array.header.frame_id.c_str());

  for (size_t i = 0; i < pose_array.poses.size(); ++i) {
    const auto & pose = pose_array.poses[i];
    RCLCPP_INFO(
      this->get_logger(),
      "Pose %zu: Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f)",
      i,
      pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w);
  }
}

}  // namespace ur10_planner

//int main(int argc, char ** argv)
//{
//  rclcpp::init(argc, argv);
//
//  rclcpp::NodeOptions options;
//  auto node = std::make_shared<ur10_planner::WaypointPublisher>(options, "waypoint_publisher");
//
//  // For now, just trigger publishing once at startup.
//  node->start_timer();
//
//  rclcpp::spin(node);
//  rclcpp::shutdown();
//  return 0;
//}