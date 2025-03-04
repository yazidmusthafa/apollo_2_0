#include <rclcpp/rclcpp.hpp>
#include <slamware_ros_sdk/srv/sync_get_stcm.hpp>
#include <fstream>
#include <vector>

class SyncGetStcmClientNode : public rclcpp::Node
{
public:
    SyncGetStcmClientNode() : Node("sync_get_stcm_client_node")
    {
        client_ = this->create_client<slamware_ros_sdk::srv::SyncGetStcm>("sync_get_stcm");

        // Try to call the service right after startup
        call_service();
    }

private:
    void call_service()
    {
        if (!client_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting. Exiting.");
            rclcpp::shutdown();
            return;
        }

        auto request = std::make_shared<slamware_ros_sdk::srv::SyncGetStcm::Request>();

        auto future = client_->async_send_request(request,
            std::bind(&SyncGetStcmClientNode::response_callback, this, std::placeholders::_1));
    }

    void response_callback(rclcpp::Client<slamware_ros_sdk::srv::SyncGetStcm>::SharedFuture future)
    {
        auto response = future.get();

        std::string filename = "stcm_data.bin";
        std::ofstream file(filename, std::ios::binary);

        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing");
            rclcpp::shutdown();
            return;
        }

        file.write(reinterpret_cast<const char*>(response->raw_stcm.data()), response->raw_stcm.size());
        file.close();

        RCLCPP_INFO(this->get_logger(), "STCM data saved to '%s' (%zu bytes)", filename.c_str(), response->raw_stcm.size());
        rclcpp::shutdown();
    }

    rclcpp::Client<slamware_ros_sdk::srv::SyncGetStcm>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncGetStcmClientNode>();
    rclcpp::spin(node);  // This allows the response callback to execute
    rclcpp::shutdown();
    return 0;
}
