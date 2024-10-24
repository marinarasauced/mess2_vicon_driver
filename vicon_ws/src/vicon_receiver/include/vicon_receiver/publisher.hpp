#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"


struct PositionStruct
{
    double translation[3];
    double rotation[4];
    std::string subject_name;
    std::string segment_name;
    std::string translation_type;
    unsigned int frame_number;

} typedef PositionStruct;


class Publisher
{
private:
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr position_publisher_;

public:
    bool is_ready = false;
    geometry_msgs::msg::Quaternion quat_diff_;
    
    rclcpp::Node *node_;
    Publisher(std::string topic_name, rclcpp::Node* node, geometry_msgs::msg::Quaternion quat_diff);

    void publish(PositionStruct p);

    geometry_msgs::msg::Quaternion multiply_two_quats(geometry_msgs::msg::Quaternion quat1, geometry_msgs::msg::Quaternion quat2);
};

#endif