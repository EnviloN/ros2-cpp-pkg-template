#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

/// @brief A template for a service client C++ Node.
class CppServiceClientNode: public rclcpp::Node {
private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr m_client;
    std::vector<std::thread> threads_;

public:
    CppServiceClientNode(): Node("dummy_client_cpp") {
        m_client = this->create_client<example_interfaces::srv::AddTwoInts>("dummy_service");
        this->WaitForServer();  // Wait until the server is available

        // Example service calls 
        // Note: This is not a great way to use this. Consider using a reusable thread pool!
        threads_.push_back(std::thread(std::bind(&CppServiceClientNode::CallServiceServer, this, 2, 2)));
        threads_.push_back(std::thread(std::bind(&CppServiceClientNode::CallServiceServer, this, 8, 1)));
        threads_.push_back(std::thread(std::bind(&CppServiceClientNode::CallServiceServer, this, 7, 9)));
    }

private:
    /// @brief Waits for the server in a loop.
    void WaitForServer() {
        while (!m_client->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for dummy server...");
    }

    /// @brief Calls service server asynchronously with given arguments.
    /// @note This blocks the thread while waiting for the server's response. Use multi-threading.
    /// @param a first argument of the service call
    /// @param b second argument of the service call
    void CallServiceServer(int a, int b) {
        // Create a request
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        // Call the service
        auto future = m_client->async_send_request(request);

        // Wait for the response from the server (Note: This blocks the thread!)
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response from server: %d + %d = %ld", a, b, response->sum);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize the ROS communication

    auto node = std::make_shared<CppServiceClientNode>();  // Initialize the node
    rclcpp::spin(node);  // Keep the node running until it is killed

    rclcpp::shutdown();  // Shutdown the ROS communication
    return 0;
}
