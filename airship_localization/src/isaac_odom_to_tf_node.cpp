#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomToTF : public rclcpp::Node
{
public:
    OdomToTF()
        : Node("odom_to_tf_publisher")
    {
        // 创建一个订阅者到 /odom 主题
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&OdomToTF::odom_callback, this, std::placeholders::_1));

        // 创建一个变换广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 创建一个 TransformStamped 消息
        geometry_msgs::msg::TransformStamped t;

        // 设置变换的头信息
        t.header.stamp = msg->header.stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "odom";

        // 设置从里程计消息中的平移
        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;

        // 设置从里程计消息中的旋转
        t.transform.rotation = msg->pose.pose.orientation;

        // 广播变换
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OdomToTF>());

    rclcpp::shutdown();

    return 0;
}