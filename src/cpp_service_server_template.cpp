#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

/// @brief A template for a service server C++ Node.
class CppServiceServerNode: public rclcpp::Node {
private:
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr m_server;

public:
    CppServiceServerNode(): Node("dummy_server_cpp") {
        m_server = this->create_service<example_interfaces::srv::AddTwoInts>(
            "dummy_service",
            std::bind(&CppServiceServerNode::CallbackServiceCall, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Dummy service server node started.");
    }

private:
    /// @brief Runs when a service has been called with a request
    /// @param request service call request
    /// @param response computed response for the request
    void CallbackServiceCall(
        const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
        const example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(),
            "Computed: %ld + %ld = %ld", request->a, request->b, response->sum);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize the ROS communication

    auto node = std::make_shared<CppServiceServerNode>();  // Initialize the node
    rclcpp::spin(node);  // Keep the node running until it is killed

    rclcpp::shutdown();  // Shutdown the ROS communication
    return 0;
}
