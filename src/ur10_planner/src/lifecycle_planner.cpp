#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// moveit includes
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

// LCR = Lifecycle Callback Return
using LCR = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("lifecycle_planner_logger");
namespace rvt = rviz_visual_tools;



class lifecycle_planner : public rclcpp_lifecycle::LifecycleNode {
    public:
        lifecycle_planner(): LifecycleNode("lifecycle_planner") {
            RCLCPP_INFO(this->get_logger(), "In constructor ... waiting for next step");
        }

        LCR on_configure(const rclcpp_lifecycle::State &previous_state) {
            (void)previous_state;
            RCLCPP_INFO(LOGGER, "IN **ON_CONFIGURE**");
            static const std::string PLANNING_GROUP = "arm";

            auto node_ptr = std::dynamic_pointer_cast<rclcpp::Node>(this->shared_from_this());

            move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_ptr, PLANNING_GROUP);            
            planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

            // initialize visualization 

            visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_ptr,"/world", "published_markers", move_group->getRobotModel());
            visual_tools->deleteAllMarkers();

            visual_tools->loadRemoteControl();

            text_pose.translation().z() = 1.0;
            visual_tools->publishText(text_pose, "lifecycle_planner (may need to change)", rvt::WHITE, rvt::XLARGE);

            visual_tools->trigger();

            // log info

            RCLCPP_INFO(LOGGER, "Ur10 planning frame          : %s", move_group->getPlanningFrame().c_str());
            RCLCPP_INFO(LOGGER, "Ur10 end effector link       : %s", move_group->getEndEffectorLink().c_str());
            RCLCPP_INFO(LOGGER, "All available planning groups: ");
            std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));

            return LCR::SUCCESS;
        };

        LCR on_activate(const rclcpp_lifecycle::State &previous_state) {
            RCLCPP_INFO(LOGGER, "IN **ON_ACTIVATE**");
            rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
            
            // planning to a pose goal

            visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to start");

            // set example pose 
            // TODO : write a function where we can parse different poses (points) and create a sudo "welding" path
            target_pose.position = move_group->getCurrentPose().pose.position;
            target_pose.orientation = move_group->getCurrentPose().pose.orientation;
            target_pose.position.x = move_group->getCurrentPose().pose.position.x + 1;
            
            move_group->setPoseTarget(target_pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;

            bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            RCLCPP_INFO(LOGGER, "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

            // visualizing plan

            RCLCPP_INFO(LOGGER, "Visualizing plan as trajectory line");

            // raw pointer to refer to the planning group
            static const std::string PLANNING_GROUP = "arm";
            const moveit::core::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

            visual_tools->publishAxisLabeled(target_pose, "test_pose");
            visual_tools->publishText(text_pose, "pose_goal", rvt::WHITE, rvt::XLARGE);
            visual_tools->publishTrajectoryLine(plan.trajectory, joint_model_group);
            visual_tools->trigger();

            return LCR::SUCCESS;
        };

        LCR on_deactivate(const rclcpp_lifecycle::State &previous_state) {
            RCLCPP_INFO(this->get_logger(), "IN **ON_DEACTIVATE**");
            rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);

            return LCR::SUCCESS;
        };

        LCR on_cleanup(const rclcpp_lifecycle::State &previous_state) {
            (void)previous_state;
            RCLCPP_INFO(this->get_logger(), "IN **ON_CLEANUP**");

            return LCR::SUCCESS;
        };

        LCR on_shutdown(const rclcpp_lifecycle::State &previous_state) {
            (void)previous_state;
            RCLCPP_INFO(this->get_logger(), "IN **ON_SHUTDOWN**");

            return LCR::SUCCESS;
        };

    private:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
        // class to add and remove collision object in our scene
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

        geometry_msgs::msg::Pose target_pose;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<lifecycle_planner>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0; 
}