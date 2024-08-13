#include "vicon_receiver/publisher.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

Publisher::Publisher(std::string topic_name, rclcpp::Node* node) : node_(node)
{
    position_publisher_ = node->create_publisher<geometry_msgs::msg::TransformStamped>(topic_name, 10);
    is_ready = true;
}

void Publisher::publish(PositionStruct p)
{
    // create message
    auto msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
    
    // fill header
    msg->header.stamp = node_->now();

    // fill transform    
    msg->transform.translation.x = p.translation[0] / 1000.0;
    msg->transform.translation.y = p.translation[1] / 1000.0;
    msg->transform.translation.z = p.translation[2] / 1000.0;
    msg->transform.rotation.x = p.rotation[0];
    msg->transform.rotation.y = p.rotation[1];
    msg->transform.rotation.z = p.rotation[2];
    msg->transform.rotation.w = p.rotation[3];
    
    // publish message
    position_publisher_->publish(*msg);
}
