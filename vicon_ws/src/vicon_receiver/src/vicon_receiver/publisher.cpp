#include "vicon_receiver/publisher.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

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
    quat_true = multiply_two_quats(quat_diff_, quat_meas);

    // fill transform    
    msg->transform.translation.x = p.translation[0] / 1000.0;
    msg->transform.translation.y = p.translation[1] / 1000.0;
    msg->transform.translation.z = p.translation[2] / 1000.0;
    msg->transform.rotation = quat_true;
    
    // publish message
    bool all_zero = true;
    if (msg->transform.translation.x != 0.0 || msg->transform.translation.y != 0.0 || msg->transform.translation.z != 0.0)
        {
            all_zero = false;
        }
    if (msg->transform.rotation.x != 0.0 || msg->transform.rotation.y != 0.0 || msg->transform.rotation.z != 0.0 || msg->transform.rotation.w != 0.0)
    {
        all_zero = false;
    }
    if (!all_zero)
    {
        position_publisher_->publish(*msg);
    }  
    // position_publisher_->publish(*msg);
}

geometry_msgs::msg::Quaternion Publisher::multiply_two_quats(geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2) {
        geometry_msgs::msg::Quaternion quat_product;
        
        quat_product.x = quat1.w * quat2.x + quat1.x * quat2.w + quat1.y * quat2.z - quat1.z * quat2.y;
        quat_product.y = quat1.w * quat2.y - quat1.x * quat2.z + quat1.y * quat2.w + quat1.z * quat2.x;
        quat_product.z = quat1.w * quat2.z + quat1.x * quat2.y - quat1.y * quat2.x + quat1.z * quat2.w;
        quat_product.w = quat1.w * quat2.w - quat1.x * quat2.x - quat1.y * quat2.y - quat1.z * quat2.z;

        return quat_product;
    }
