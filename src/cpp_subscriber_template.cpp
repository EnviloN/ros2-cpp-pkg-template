#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using std::placeholders::_1;

/// @brief A template for a subscriber C++ Node.
class CppSubscriberTemplateNode: public rclcpp::Node {
private:
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr m_subscriber;

public:
    CppSubscriberTemplateNode(): Node("dummy_publisher_cpp") {
        m_subscriber = this->create_subscription<example_interfaces::msg::String>(
            "dummy_topic", 10, std::bind(&CppSubscriberTemplateNode::CallbackMsgReceived, this, _1));

        RCLCPP_INFO(this->get_logger(), "Dummy publisher node started.");
    }

private:
    /// @brief Runs when a message is received.
    void CallbackMsgReceived(const example_interfaces::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize the ROS communication

    auto node = std::make_shared<CppSubscriberTemplateNode>();  // Initialize the node
    rclcpp::spin(node);  // Keep the node running until it is killed

    rclcpp::shutdown();  // Shutdown the ROS communication
    return 0;
}
