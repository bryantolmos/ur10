#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// LCR = Lifecycle Callback Return
using LCR = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class lifecycle_planner : public rclcpp_lifecycle::LifecycleNode {
    public:
        lifecycle_planner(): LifecycleNode("lifecycle_planner") {
            RCLCPP_INFO(this->get_logger(), "In constructor ... waiting for next step");
        }

        LCR on_configure(const rclcpp_lifecycle::State &previous_state) {
            (void)previous_state;
            RCLCPP_INFO(this->get_logger(), "IN **ON_CONFIGURE**");

            return LCR::SUCCESS;
        };

        LCR on_activate(const rclcpp_lifecycle::State &previous_state) {
            RCLCPP_INFO(this->get_logger(), "IN **ON_ACTIVATE**");
            rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);

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

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lifecycle_planner>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0; 
}