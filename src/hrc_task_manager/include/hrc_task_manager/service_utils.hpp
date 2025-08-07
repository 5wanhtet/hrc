#ifndef HRC_TASK_MANAGER_SERVICE_UTILS_HPP
#define HRC_TASK_MANAGER_SERVICE_UTILS_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <memory>
#include <string>
#include <chrono>
#include <thread>

namespace hrc_task_manager
{

class ServiceUtils
{
public:
    // Initialize the service client (call this once in main)
    static void initialize(std::shared_ptr<rclcpp::Node> node);
    
    // Call the service with a command
    static bool call_service(const std::string& command);
    
    // Wait for service to be available
    static bool wait_for_service(std::chrono::seconds timeout = std::chrono::seconds(10));
    
    // Utility function for delay
    static void delay_ms(int ms);
    
    // Check if service is initialized
    static bool is_initialized();
    
    // Shutdown (optional, for cleanup)
    static void shutdown();

private:
    static std::shared_ptr<rclcpp::Node> node_;
    static rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    static bool initialized_;
};

} // namespace hrc_task_manager

#endif // HRC_TASK_MANAGER_SERVICE_UTILS_HPP