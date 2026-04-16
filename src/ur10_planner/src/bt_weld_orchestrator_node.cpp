#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "ur10_planner/action/weld.hpp"

using namespace BT;

// Node 1: Call the Path Generator Service
class GeneratePathNode : public RosServiceNode<std_srvs::srv::Trigger> {
public:
    GeneratePathNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosServiceNode<std_srvs::srv::Trigger>(name, conf, params) {}

    static PortsList providedPorts() { return {}; }

    bool setRequest(Request::SharedPtr& request) override {
        (void)request; return true; 
    }

    NodeStatus onResponseReceived(const Response::SharedPtr& response) override {
        if (response->success) {
            // FIX: Added .lock() to access the weak_ptr
            RCLCPP_INFO(node_.lock()->get_logger(), "BT: Path generated successfully.");
            return NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(node_.lock()->get_logger(), "BT: Path generation failed: %s", response->message.c_str());
        return NodeStatus::FAILURE;
    }
};

// Node 2: Call the Execute Weld Action Server
class ExecuteWeldNode : public RosActionNode<ur10_planner::action::Weld> {
public:
    ExecuteWeldNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<ur10_planner::action::Weld>(name, conf, params) {}

    static PortsList providedPorts() { return {}; }

    bool setGoal(RosActionNode::Goal& goal) override {
        goal.execute = true;
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult& wr) override {
        if (wr.result->success) {
            RCLCPP_INFO(node_.lock()->get_logger(), "BT: Weld finished successfully.");
            return NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(node_.lock()->get_logger(), "BT: Weld failed.");
        return NodeStatus::FAILURE;
    }

    virtual NodeStatus onFailure(ActionNodeErrorCode error) override {
        RCLCPP_ERROR(node_.lock()->get_logger(), "BT: Action Client Error %d", error);
        return NodeStatus::FAILURE;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_weld_orchestrator");

    node->declare_parameter<std::string>("bt_xml_file", "");
    std::string xml_file = node->get_parameter("bt_xml_file").as_string();

    BehaviorTreeFactory factory;
    
    // BT.CPP v4: Must pass the ROS node pointer into the Factory via RosNodeParams
    RosNodeParams params;
    params.nh = node;
    
    // FIX 1: Give ROS 2 DDS time to discover the network (Wait up to 5 seconds instead of 500ms)
    params.server_timeout = std::chrono::milliseconds(5000); 

    // FIX 2: Use absolute paths (leading slash) so it doesn't get lost in namespaces
    params.default_port_value = "/path_generator_node/generate_path";
    factory.registerNodeType<GeneratePathNode>("GeneratePath", params);

    params.default_port_value = "/execute_weld";
    factory.registerNodeType<ExecuteWeldNode>("ExecuteWeld", params);

    auto tree = factory.createTreeFromFile(xml_file);
    RCLCPP_INFO(node->get_logger(), "Behavior Tree Loaded. Ticking...");

    // Tick the tree until it finishes
    NodeStatus status = NodeStatus::RUNNING;
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickExactlyOnce();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node->get_logger(), "Behavior Tree Finished.");
    rclcpp::shutdown();
    return 0;
}