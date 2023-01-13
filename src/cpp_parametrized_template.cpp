#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"


/// @brief A template for a simple parametrized publisher C++ Node.
class CppParametrizedTemplateNode: public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr m_publisher;

    u_int64_t m_time_delta_ms;
    std::string m_word;

public:
    CppParametrizedTemplateNode(): Node("dummy_parametrized_cpp") {
        // Declaring a parameter with default value
        // The type is inferred from default value
        this->declare_parameter("time_delta_ms", 1000);
        this->declare_parameter("word", "BAZINGA");

        // Get values from parameters (can be done when the parameters are needed)
        m_time_delta_ms = this->get_parameter("time_delta_ms").get_parameter_value().get<u_int64_t>();
        m_word = this->get_parameter("word").get_parameter_value().get<std::string>();

        m_publisher = this->create_publisher<example_interfaces::msg::String>("dummy_topic", 10);
        m_timer = this->create_wall_timer(
            std::chrono::milliseconds(m_time_delta_ms),
            std::bind(&CppParametrizedTemplateNode::PublishMessage, this)
        );

        RCLCPP_INFO(this->get_logger(), "Parametrized dummy publisher node started.");
    }

private:
    /// @brief Publishes a message with parametrized content.
    void PublishMessage() {
        auto msg = example_interfaces::msg::String();
        msg.data = "Hello, here is the secret word: " + m_word;
        m_publisher->publish(msg);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize the ROS communication

    auto node = std::make_shared<CppParametrizedTemplateNode>();  // Initialize the node
    rclcpp::spin(node);  // Keep the node running until it is killed

    rclcpp::shutdown();  // Shutdown the ROS communication
    return 0;
}
