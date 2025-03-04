#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class SetRobotPoseNode : public rclcpp::Node
{
public:
    SetRobotPoseNode() : Node("set_robot_pose")
    {
        initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&SetRobotPoseNode::setPoseCallback, this, std::placeholders::_1)
        );

        set_pose_pub_ = create_publisher<geometry_msgs::msg::Pose>("/slamware_ros_sdk_server_node/set_pose", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr set_pose_pub_;

    void setPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {   
        geometry_msgs::msg::Pose pose_msg = msg->pose.pose;  // Extract Pose

        set_pose_pub_->publish(pose_msg);  // Publish Pose (not the full PoseWithCovarianceStamped)

        RCLCPP_INFO(this->get_logger(), "Robot Current Pose set Completed");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetRobotPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
