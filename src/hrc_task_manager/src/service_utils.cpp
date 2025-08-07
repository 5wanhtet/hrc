#include "hrc_task_manager/service_utils.hpp"
#include <iostream>

namespace hrc_task_manager
{

// Static member definitions
std::shared_ptr<rclcpp::Node> ServiceUtils::node_ = nullptr;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ServiceUtils::client_ = nullptr;
bool ServiceUtils::initialized_ = false;

// Function to initialize client node (trigger) -> bt_commands_service
void ServiceUtils::initialize(std::shared_ptr<rclcpp::Node> node)
{
    node_ = node;
    client_ = node_->create_client<std_srvs::srv::Trigger>("bt_commands_service");
    initialized_ = true;
    
    std::cout << "ServiceUtils initialized with node: " << node_->get_name() << std::endl;
}

// Function to request service, respond with success if response received before timeout
bool ServiceUtils::call_service(const std::string& command)
{
    if (!initialized_ || !client_) {
        std::cout << "ERROR: ServiceUtils not initialized or client not available!\n";
        return false;
    }

    std::cout << "Calling service with command: " << command << std::endl;
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        std::cout << "SUCCESS: Service response: " << response->message << std::endl;
        return response->success;
    } else {
        std::cout << "FAIL: Service call failed or timed out for command: " << command << std::endl;
        return false;
    }
}

// waiting for dummy node to start and connect to dummy node
bool ServiceUtils::wait_for_service(std::chrono::seconds timeout)
{
    if (!initialized_ || !client_) {
        std::cout << "ERROR: ServiceUtils not initialized!\n";
        return false;
    }
    
    std::cout << "Waiting for 'bt_commands_service' to be available";
    auto start_time = std::chrono::steady_clock::now();
    
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            std::cout << "\nInterrupted while waiting for service\n";
            return false;
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout) {
            std::cout << "\nTimeout waiting for service\n";
            return false;
        }
        
        std::cout << ".";
        std::cout.flush();
    }
    
    std::cout << " ✓ Service found!\n";
    return true;
}

void ServiceUtils::delay_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

bool ServiceUtils::is_initialized()
{
    return initialized_;
}

void ServiceUtils::shutdown()
{
    client_.reset();
    node_.reset();
    initialized_ = false;
    std::cout << "ServiceUtils shutdown complete\n";
}

} // namespace hrc_task_manager