#include "vicon_receiver/publisher.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mess2_plugins/orientation.hpp"

Publisher::Publisher(std::string topic_name, rclcpp::Node* node, geometry_msgs::msg::Quaternion quat_diff) : node_(node)
{
    position_publisher_ = node->create_publisher<geometry_msgs::msg::TransformStamped>(topic_name, 10);
    is_ready = true;
    quat_diff_ = quat_diff;
}

void Publisher::publish(PositionStruct p)
{
    // create message
    auto msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
    
    // fill header
    msg->header.stamp = node_->now();

    geometry_msgs::msg::Quaternion quat_meas;
    quat_meas.x = p.rotation[0];
    quat_meas.y = p.rotation[1];
    quat_meas.z = p.rotation[2];
    quat_meas.w = p.rotation[3];

    geometry_msgs::msg::Quaternion quat_true;
    quat_true = mess2_plugins::multiply_two_quats(quat_diff_, quat_meas);

    // fill transform    
    msg->transform.translation.x = p.translation[0] / 1000.0;
    msg->transform.translation.y = p.translation[1] / 1000.0;
    msg->transform.translation.z = p.translation[2] / 1000.0;
    msg->transform.rotation = quat_true;
    
    // publish message
    // bool all_zero = true;
    // if (msg->transform.translation.x != 0.0 || msg->transform.translation.y != 0.0 || msg->transform.translation.z != 0.0)
    //     {
    //         all_zero = false;
    //     }
    // if (msg->transform.rotation.x != 0.0 || msg->transform.rotation.y != 0.0 || msg->transform.rotation.z != 0.0 || msg->transform.rotation.w != 0.0)
    // {
    //     all_zero = false;
    // }
    // if (!all_zero)
    // {
    //     position_publisher_->publish(*msg);
    // }  
    position_publisher_->publish(*msg);
}
