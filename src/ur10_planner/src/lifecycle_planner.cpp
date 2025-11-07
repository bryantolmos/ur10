// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

// c++
#include <thread>

// moveit includes
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

// moveit msgs
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

// utilities
#include "../include/ur10_planner/waypoint_publisher.hpp"

#include <std_srvs/srv/trigger.hpp>

// LCR = Lifecycle Callback Return
using LCR = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace rvt = rviz_visual_tools;


class lifecycle_planner : public rclcpp_lifecycle::LifecycleNode {
    public:
        lifecycle_planner(const rclcpp::NodeOptions & options) : 
        LifecycleNode("lifecycle_planner", options),
        PLANNING_GROUP("arm"),
        END_EFFECTOR("welding_torch_end_effector"),
        PATH_TOPIC("/welding_path"),
        SERVICE("~/plan_and_execute"),
        TRAJECTORY_TOPIC("/planned_trajectory")
        {
            RCLCPP_INFO(this->get_logger(), "In constructor ... waiting for next step");
        }

        LCR on_configure(const rclcpp_lifecycle::State &previous_state) {
            (void)previous_state;
            RCLCPP_INFO(this->get_logger(), "IN **ON_CONFIGURE**");
            
            // create and start moveit node 
            // moveit rquires a regular rclcpp::Node, not a rclcpp_lifecycle::LifecycleNode, so we create a new node
            // then we spin this regular node own its own thread. We then pass in the options from the lifecycle node
            // which is how the regular node gets all the moveit parameters.
            auto node_options = this->get_node_options();
            moveit_node = std::make_shared<rclcpp::Node>("lifecycle_planner_moveit_node", node_options);

            // initialize waypoint publisher node
            waypoint_publisher_node = std::make_shared<ur10_planner::WaypointPublisher>(node_options);

            // add both nodes into a seperate thread
            executor_thread = std::thread([this]() {
                this->executor.add_node(this->moveit_node);
                this->executor.add_node(this->waypoint_publisher_node);
                this->executor.spin();
            });
            RCLCPP_INFO(this->get_logger(), "node thread started");

            // start and check MoveGroupInterface initialization
            move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node, PLANNING_GROUP);
            if(!move_group) {
                RCLCPP_FATAL(this->get_logger(), "FAILED to initialize MoveGroupInterface class");
                return LCR::FAILURE;
            }            
            RCLCPP_INFO(this->get_logger(), "SUCCESS initializing MoveGroupInterface class");
            
            // check for a valid robot model
            if(!move_group->getRobotModel()) {
                RCLCPP_FATAL(this->get_logger(), "FAILED to get robot model");
                return LCR::FAILURE;
            }
            RCLCPP_INFO(this->get_logger(), "SUCCESS retreiving robot model");

            // set the end effector from the robots SRDF
            move_group->setEndEffector(END_EFFECTOR);

            // initialize path_subscriber using transient local qos
            //auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
            auto qos = rclcpp::SystemDefaultsQoS();

            path_subscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
                PATH_TOPIC,
                qos,
                std::bind(&lifecycle_planner::path_callback, this, std::placeholders::_1)
            );

            // initialize trajectory publisher
            trajectory_publisher = this->create_publisher<moveit_msgs::msg::RobotTrajectory>(
                TRAJECTORY_TOPIC,
                rclcpp::SystemDefaultsQoS()
            );

            // initialize service
            trigger_planning_service = this->create_service<std_srvs::srv::Trigger>(
                SERVICE,
                std::bind(&lifecycle_planner::planning_callback, this, std::placeholders::_1, std::placeholders::_2)
            );

            // initialize planning scene and visual tools
            planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
            visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(moveit_node,"/world", "published_markers", move_group->getRobotModel());

            visual_tools->deleteAllMarkers();
            visual_tools->loadRemoteControl();

            text_pose.translation().z() = 2.0;
            visual_tools->publishText(text_pose, "lifecycle_planner : Configured", rvt::WHITE, rvt::XXLARGE);
            visual_tools->trigger();

            // log info
            RCLCPP_INFO(this->get_logger(), "Ur10 planning frame: %s", move_group->getPlanningFrame().c_str());
            RCLCPP_INFO(this->get_logger(), "Ur10 end effector link: %s", move_group->getEndEffectorLink().c_str());
            RCLCPP_INFO(this->get_logger(), "Configuration successful, ready to move to activate");

            return LCR::SUCCESS;
        };

        LCR on_activate(const rclcpp_lifecycle::State &previous_state) {
            RCLCPP_INFO(this->get_logger(), "IN **ON_ACTIVATE**");
            rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
            
            received_waypoints.clear();

            trajectory_publisher->on_activate();

            return LCR::SUCCESS;
        };

        LCR on_deactivate(const rclcpp_lifecycle::State &previous_state) {
            RCLCPP_INFO(this->get_logger(), "IN **ON_DEACTIVATE**");

            if(move_group) {
                move_group->stop();
            }

            received_waypoints.clear();

            trajectory_publisher->on_deactivate();

            rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
            return LCR::SUCCESS;
        };

        LCR on_cleanup(const rclcpp_lifecycle::State &previous_state) {
            (void)previous_state;
            RCLCPP_INFO(this->get_logger(), "IN **ON_CLEANUP**");

            // stop executor
            executor.cancel();

            // join thread
            // join the thread and wait for it to finish executing
            if(executor_thread.joinable()) {
                executor_thread.join();
            }

            // reset smart pointers
            move_group.reset();
            planning_scene_interface.reset();
            visual_tools.reset();
            moveit_node.reset();
            waypoint_publisher_node.reset();

            path_subscriber.reset();
            trigger_planning_service.reset();
            trajectory_publisher.reset();

            RCLCPP_INFO(this->get_logger(), "SUCCESSFUL cleanup");
            return LCR::SUCCESS;
        };

        LCR on_shutdown(const rclcpp_lifecycle::State &previous_state) {
            RCLCPP_INFO(this->get_logger(), "IN **ON_SHUTDOWN**");
            return on_cleanup(previous_state);
        };

    private:
        void path_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            // check if node is active
            if(this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                RCLCPP_WARN(this->get_logger(), "Path publisher not active, waiting ...");
                return;
            }
            
            // store data
            this->received_waypoints.clear();
            this->received_waypoints = msg->poses;

            RCLCPP_INFO(this->get_logger(), "Received and store new path with %zu waypoints", this->received_waypoints.size());
        }

        void planning_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void) request;

                // check if node is active
                if(this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    RCLCPP_ERROR(this->get_logger(), "service called while node is not active");
                    response->success = false;
                    response->message = "Planner not active";
                    return;
                }

                // check if we have waypoints
                if(this->received_waypoints.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "Service called but no path received");
                    response->success = false;
                    response->message = "No path received";
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "Planning service called with %zu waypoints", this->received_waypoints.size());

                // create trajectory object then compute cartesian path
                moveit_msgs::msg::RobotTrajectory trajectory;

                double fraction = move_group->computeCartesianPath(
                    this->received_waypoints, // input path to follow
                    0.01, // ee step, 1cm
                    trajectory // output
                );

                RCLCPP_INFO(this->get_logger(), "Cartesian path (%.2f%%) achieved", fraction * 100);

                if (fraction < 0.97) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to compute full Cartesian path");
                    response->success = false;
                    response->message = "Failed to compute full cartesian path";
                    return;
                }

                // visualize trajectory
                const std::string ee_link_name = move_group->getEndEffectorLink();
                const moveit::core::LinkModel* ee_link_model = move_group->getRobotModel()->getLinkModel(ee_link_name);
                const moveit::core::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

                visual_tools->publishTrajectoryLine(trajectory, ee_link_model, joint_model_group);
                visual_tools->trigger();

                // publish computed trajectory
                trajectory_publisher->publish(trajectory);
                RCLCPP_INFO(this->get_logger(), "Planning successful, trajectory published to /planned_trajectory");

                // clean up the path
                this->received_waypoints.clear();

                response->success = true;
                response->message = "Plan published for validation";
            }

        // configuration variables
        const std::string PLANNING_GROUP;
        const std::string END_EFFECTOR;
        const std::string PATH_TOPIC;
        const std::string SERVICE;
        const std::string TRAJECTORY_TOPIC;

        // moveit
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;

        // thread
        rclcpp::Node::SharedPtr moveit_node;
        std::shared_ptr<ur10_planner::WaypointPublisher> waypoint_publisher_node;
        rclcpp::executors::SingleThreadedExecutor executor;
        std::thread executor_thread;

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_subscriber;
        rclcpp_lifecycle::LifecyclePublisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_publisher;
        std::vector<geometry_msgs::msg::Pose> received_waypoints;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_planning_service;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // allow parameters from launch file to be loaded
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<lifecycle_planner>(options);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0; 
}