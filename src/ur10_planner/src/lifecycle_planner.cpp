// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

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

// LCR = Lifecycle Callback Return
using LCR = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace rvt = rviz_visual_tools;


class lifecycle_planner : public rclcpp_lifecycle::LifecycleNode {
    public:
        lifecycle_planner(const rclcpp::NodeOptions & options) : 
        LifecycleNode("lifecycle_planner", options),
        PLANNING_GROUP("arm"),
        END_EFFECTOR("welding_torch_end_effector")
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

            executor_thread = std::thread([this]() {
                this->executor.add_node(this->moveit_node);
                this->executor.spin();
            });
            RCLCPP_INFO(this->get_logger(), "MoveIt node thread started");

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
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            
            visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to start");

            // set target pose
            //target_pose.position = move_group->getCurrentPose().pose.position;
            //target_pose.orientation = move_group->getCurrentPose().pose.orientation;
            //target_pose.position.x = move_group->getCurrentPose().pose.position.x + 0.5;
            //target_pose.position.y = move_group->getCurrentPose().pose.position.y + 0.8;
            //target_pose.position.z = move_group->getCurrentPose().pose.position.z + 0.6;
            target_pose = move_group->getCurrentPose().pose;
            target_pose.position.x += 0.5;
            target_pose.position.z += 0.3;

            move_group->setPoseTarget(target_pose);

            // plan out
            bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            RCLCPP_INFO(this->get_logger(), "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

            if(!success) {
                return LCR::FAILURE;
            }

            // visualizing plan
            RCLCPP_INFO(this->get_logger(), "Visualizing plan as trajectory line");
            const std::string ee_link_name = move_group->getEndEffectorLink();
            const moveit::core::LinkModel* ee_link_model = move_group->getRobotModel()->getLinkModel(ee_link_name);
            const moveit::core::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

            visual_tools->publishAxisLabeled(target_pose, "test_pose");
            visual_tools->publishText(text_pose, "pose_goal", rvt::WHITE, rvt::XLARGE);
            visual_tools->publishTrajectoryLine(plan.trajectory, ee_link_model, joint_model_group);
            visual_tools->trigger();

            // execute plan
            visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to move the robot along visualized trajectory");
            move_group->move();

            return LCR::SUCCESS;
        };

        LCR on_deactivate(const rclcpp_lifecycle::State &previous_state) {
            RCLCPP_INFO(this->get_logger(), "IN **ON_DEACTIVATE**");

            if(move_group) {
                move_group->stop();
            }

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

            RCLCPP_INFO(this->get_logger(), "SUCCESSFUL cleanup");
            return LCR::SUCCESS;
        };

        LCR on_shutdown(const rclcpp_lifecycle::State &previous_state) {
            RCLCPP_INFO(this->get_logger(), "IN **ON_SHUTDOWN**");
            return on_cleanup(previous_state);
        };

    private:
        // configuration variables
        const std::string PLANNING_GROUP;
        const std::string END_EFFECTOR;

        // moveit
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;

        // thread
        rclcpp::Node::SharedPtr moveit_node;
        rclcpp::executors::SingleThreadedExecutor executor;
        std::thread executor_thread;

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        geometry_msgs::msg::Pose target_pose;
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