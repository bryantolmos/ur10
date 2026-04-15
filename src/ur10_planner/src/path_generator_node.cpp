#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>

class PathGeneratorNode : public rclcpp::Node {
public:
    PathGeneratorNode() : Node("path_generator_node") {
        this->declare_parameter<double>("weave_amplitude", 0.05);
        this->declare_parameter<double>("weave_step_length", 0.02);
        this->declare_parameter<std::string>("points_file_path", "");
        this->declare_parameter<std::string>("frame_id", "world");

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/welding_path", rclcpp::QoS(1).transient_local());

        // Publish once after a short delay to ensure subscribers are ready
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&PathGeneratorNode::generate_and_publish, this));
    }

private:
    void generate_and_publish() {
        timer_->cancel(); // One-shot
        
        std::string file_path = this->get_parameter("points_file_path").as_string();
        double amplitude = this->get_parameter("weave_amplitude").as_double();
        double step = this->get_parameter("weave_step_length").as_double();
        
        std::vector<geometry_msgs::msg::Pose> raw_points;
        
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            const YAML::Node& points = config["selected_points"];
            for (std::size_t i = 0; i < points.size(); ++i) {
                geometry_msgs::msg::Pose p;
                p.position.x = points[i]["x"].as<double>();
                p.position.y = points[i]["y"].as<double>();
                p.position.z = points[i]["z"].as<double>();
                // Torch pointing straight down (Pitch = PI)
                tf2::Quaternion q; q.setRPY(0.0, M_PI, 0.0);
                p.orientation = tf2::toMsg(q);
                raw_points.push_back(p);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
            return;
        }

        if (raw_points.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Need at least 2 points to generate a path.");
            return;
        }

        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = this->get_parameter("frame_id").as_string();

        for (size_t i = 0; i < raw_points.size() - 1; ++i) {
            add_weave_segment(msg, raw_points[i], raw_points[i+1], amplitude, step);
        }
        msg.poses.push_back(raw_points.back());

        RCLCPP_INFO(this->get_logger(), "Publishing %zu woven waypoints to /welding_path", msg.poses.size());
        publisher_->publish(msg);
    }

    void add_weave_segment(geometry_msgs::msg::PoseArray& msg, const geometry_msgs::msg::Pose& p_start, const geometry_msgs::msg::Pose& p_end, double amp, double step) {
        tf2::Transform tf_a, tf_b;
        tf2::fromMsg(p_start, tf_a); tf2::fromMsg(p_end, tf_b);
        
        tf2::Vector3 path_vector = tf_b.getOrigin() - tf_a.getOrigin();
        double dist = path_vector.length();
        if (dist < 1e-6) return;

        tf2::Vector3 dir = path_vector.normalized();
        tf2::Vector3 weave_dir = dir.cross(tf2::Vector3(0, 0, 1)).normalized();

        int steps = std::max(2, static_cast<int>(dist / step));
        for (int i = 0; i < steps; ++i) {
            double t = static_cast<double>(i) / (steps - 1);
            tf2::Vector3 interp_pos = tf_a.getOrigin().lerp(tf_b.getOrigin(), t);
            
            double offset = (i % 2 == 1) ? amp : -amp;
            if (i == 0 || i == steps - 1) offset = 0.0;

            geometry_msgs::msg::Pose p;
            p.position.x = interp_pos.x() + (weave_dir.x() * offset);
            p.position.y = interp_pos.y() + (weave_dir.y() * offset);
            p.position.z = interp_pos.z() + (weave_dir.z() * offset);
            p.orientation = tf2::toMsg(tf_a.getRotation().slerp(tf_b.getRotation(), t));
            msg.poses.push_back(p);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}