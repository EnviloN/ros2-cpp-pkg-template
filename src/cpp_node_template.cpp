#include "rclcpp/rclcpp.hpp"


/// @brief A template for a basic C++ Node.
class CppTemplateNode: public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    int m_counter = 0;

public:
    CppTemplateNode(): Node("dummy_cpp"), m_counter(0) {
        RCLCPP_INFO(this->get_logger(), "Hello ROS2!");
        m_timer = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&CppTemplateNode::TimerCallback, this)
        );
    }

private:
    /// @brief Increments a counter and logs a message.
    void TimerCallback() {
        m_counter++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", m_counter);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize the ROS communication

    auto node = std::make_shared<CppTemplateNode>();  // Initialize the node
    rclcpp::spin(node);  // Keep the node running until it is killed

    rclcpp::shutdown();  // Shutdown the ROS communication
    return 0;
}
