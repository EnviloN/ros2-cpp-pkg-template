#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"


/// @brief A template for a publisher C++ Node.
class CppPublisherTemplateNode: public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr m_publisher;

public:
    CppPublisherTemplateNode(): Node("dummy_publisher_cpp") {
        m_publisher = this->create_publisher<example_interfaces::msg::String>("dummy_topic", 10);
        m_timer = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&CppPublisherTemplateNode::PublishMessage, this)
        );

        RCLCPP_INFO(this->get_logger(), "Dummy publisher node started.");
    }

private:
    /// @brief Publishes a message.
    void PublishMessage() {
        auto msg = example_interfaces::msg::String();
        msg.data = "Hello from a dummy publisher!";
        m_publisher->publish(msg);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize the ROS communication

    auto node = std::make_shared<CppPublisherTemplateNode>();  // Initialize the node
    rclcpp::spin(node);  // Keep the node running until it is killed

    rclcpp::shutdown();  // Shutdown the ROS communication
    return 0;
}
