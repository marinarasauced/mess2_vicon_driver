#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "publisher.hpp"
#include <iostream>
#include <map>
#include <chrono>
#include <string>
#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

using namespace std;

class Communicator : public rclcpp::Node
{
private:
    ViconDataStreamSDK::CPP::Client vicon_client;
    string hostname;
    unsigned int buffer_size;
    string ns_name;
    map<string, Publisher> pub_map;
    boost::mutex mutex;
    string actors_dir;

public:
    Communicator();

    bool connect();

    bool disconnect();

    void get_frame();

    void create_publisher(const string subject_name, const string segment_name);
    void create_publisher_thread(const string subject_name, const string segment_name);

    geometry_msgs::msg::Quaternion normalize_quat(geometry_msgs::msg::Quaternion quat);
};

#endif // COMMUNICATOR_HPP
