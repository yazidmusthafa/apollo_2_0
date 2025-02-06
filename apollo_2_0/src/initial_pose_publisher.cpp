#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>  // for quaternion math
#include <cmath>
#include <vector>
#include <array>
#include <functional>
#include <memory>

class ApolloInitialPose : public rclcpp::Node
{
    public:

    ApolloInitialPose() : Node("apollo_initial_pose_node")
    {
        initial_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        // Subscriber to check if pose is set
        pose_check_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10, 
                            std::bind(&ApolloInitialPose::pose_set_callback, this, std::placeholders::_1));

        set_initial_pose();
    }

    private:

    void set_initial_pose()
    {   
        // Create a quaternion object
        tf2::Quaternion q;
        q.setX(0.0);
        q.setY(0.0);
        q.setZ(-0.707);
        q.setW(0.707);

        // Normalize the quaternion
        q.normalize();  // This ensures it is valid

        geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
        init_pose.header.frame_id = "map";
        init_pose.pose.pose.position.x = 0.0;
        init_pose.pose.pose.position.y = 0.0;
        init_pose.pose.pose.position.z = 0.0;

        // Assign the normalized quaternion to the Pose
        init_pose.pose.pose.orientation.x = q.x();
        init_pose.pose.pose.orientation.y = q.y();
        init_pose.pose.pose.orientation.z = q.z();
        init_pose.pose.pose.orientation.w = q.w();

        rclcpp::Rate loop_rate(4);

        while(!pose_set_ && rclcpp::ok())
        {
            initial_pose->publish(init_pose);
            rclcpp::spin_some(this->get_node_base_interface()); // Process callbacks
            loop_rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Pose set, shutting down node.");
        rclcpp::shutdown();  // Shutdown ROS 2 once pose is set

    }

    void pose_set_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Initial pose successfully set!");
        pose_set_ = true;
    }

    bool pose_set_ = false;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_check_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto apollo_initial_pose_node = std::make_shared<ApolloInitialPose>();

    rclcpp::spin(apollo_initial_pose_node);
    return 0;
}