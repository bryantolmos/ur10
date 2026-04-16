#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

class PathGeneratorNode : public rclcpp::Node {
public:
    PathGeneratorNode(const rclcpp::NodeOptions& options) 
    : Node("path_generator_node", options) {
        
        if (!this->has_parameter("weave_amplitude")) this->declare_parameter<double>("weave_amplitude", 0.01);
        if (!this->has_parameter("weave_step_length")) this->declare_parameter<double>("weave_step_length", 0.01);
        if (!this->has_parameter("points_file_path")) this->declare_parameter<std::string>("points_file_path", "");
        
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/welding_path", rclcpp::QoS(1).transient_local());

        generate_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/generate_path",
            std::bind(&PathGeneratorNode::generate_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Path Generator Service Ready.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr generate_service_;

    void generate_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        try {
            std::string file_path = this->get_parameter("points_file_path").as_string();
            double amplitude = this->get_parameter("weave_amplitude").as_double();
            double step = this->get_parameter("weave_step_length").as_double();

            YAML::Node config = YAML::LoadFile(file_path);
            const YAML::Node& points = config["selected_points"];
            
            std::vector<geometry_msgs::msg::Pose> raw_points;
            for (std::size_t i = 0; i < points.size(); ++i) {
                geometry_msgs::msg::Pose p;
                p.position.x = points[i]["x"].as<double>();
                p.position.y = points[i]["y"].as<double>();
                p.position.z = points[i]["z"].as<double>();
                tf2::Quaternion q; q.setRPY(0, M_PI, 0); // Pointing down
                p.orientation = tf2::toMsg(q);
                raw_points.push_back(p);
            }

            geometry_msgs::msg::PoseArray msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "world";

            for (size_t i = 0; i < raw_points.size() - 1; ++i) {
                add_weave_segment(msg, raw_points[i], raw_points[i+1], amplitude, step);
            }
            msg.poses.push_back(raw_points.back());

            publisher_->publish(msg);
            response->success = true;
            response->message = "Generated " + std::to_string(msg.poses.size()) + " points.";
            RCLCPP_INFO(this->get_logger(), "Published path!");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = e.what();
            RCLCPP_ERROR(this->get_logger(), "Generation failed: %s", e.what());
        }
    }

    void add_weave_segment(geometry_msgs::msg::PoseArray& msg, const geometry_msgs::msg::Pose& p_start,
                           const geometry_msgs::msg::Pose& p_end, double amp, double step) {
        tf2::Transform tf_a, tf_b;
        tf2::fromMsg(p_start, tf_a); tf2::fromMsg(p_end, tf_b);
        tf2::Vector3 path_vector = tf_b.getOrigin() - tf_a.getOrigin();
        double dist = path_vector.length();
        if (dist < 1e-6) return;

        tf2::Vector3 weave_dir = path_vector.normalized().cross(tf2::Vector3(0, 0, 1)).normalized();
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
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<PathGeneratorNode>(options));
    rclcpp::shutdown();
    return 0;
}