#include <rclcpp/rclcpp.hpp>
#include <slamware_ros_sdk/srv/sync_set_stcm.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <fstream>
#include <vector>

class SyncSetStcmClientNode : public rclcpp::Node
{
public:
    SyncSetStcmClientNode() : Node("sync_set_stcm_client_node")
    {
        client_ = this->create_client<slamware_ros_sdk::srv::SyncSetStcm>("sync_set_stcm");

        if (!client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available after waiting. Exiting.");
            rclcpp::shutdown();
            return;
        }

        if (!load_and_send_map()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load or send map data.");
            rclcpp::shutdown();
        }
    }

private:
    bool load_and_send_map()
    {
        // Read the binary map file
        std::ifstream file("stcm_data.bin", std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open stcm_data.bin for reading");
            return false;
        }

        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<uint8_t> buffer(size);
        if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read stcm_data.bin");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded map file with %ld bytes", static_cast<long>(buffer.size()));

        // Prepare request
        auto request = std::make_shared<slamware_ros_sdk::srv::SyncSetStcm::Request>();
        request->raw_stcm = buffer;

        // Example initial pose (adjust to fit your use case)
        request->robot_pose.position.x = 0.0;
        request->robot_pose.position.y = 0.0;
        request->robot_pose.position.z = 0.0;
        request->robot_pose.orientation.x = 0.0;
        request->robot_pose.orientation.y = 0.0;
        request->robot_pose.orientation.z = 0.0;
        request->robot_pose.orientation.w = 1.0;

        // Call service
        auto future = client_->async_send_request(request);
        auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

        if (result != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Service call to set map failed.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Map successfully uploaded to robot.");
        rclcpp::shutdown();  // Shutdown after completing the task
        return true;
    }

    rclcpp::Client<slamware_ros_sdk::srv::SyncSetStcm>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncSetStcmClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
