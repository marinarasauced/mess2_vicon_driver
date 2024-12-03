#include <yaml-cpp/yaml.h>
#include "vicon_driver/communicator.hpp"


using namespace ViconDataStreamSDK::CPP;

Communicator::Communicator() : Node("vicon")
{
    this->declare_parameter<std::string>("hostname", "127.0.0.1");
    this->declare_parameter<int>("buffer_size", 200);
    this->declare_parameter<std::string>("namespace", "vicon");
    this->declare_parameter<std::string>("path_to_calibrations", "/home/mess2/mess2/actors");
    this->declare_parameter<double>("fps", 40.0);
    this->get_parameter("hostname", hostname);
    this->get_parameter("buffer_size", buffer_size);
    this->get_parameter("namespace", ns_name);
    this->get_parameter("path_to_calibrations", calibrations);
    this->get_parameter("fps", fps_);
}

bool Communicator::connect()
{
    string msg = "Connecting to " + hostname + " ...";
    cout << msg << endl;
    int counter = 0;
    while (!vicon_client.IsConnected().Connected)
    {
        bool ok = (vicon_client.Connect(hostname).Result == Result::Success);
        if (!ok)
        {
            counter++;
            msg = "Connect failed, reconnecting (" + std::to_string(counter) + ")...";
            cout << msg << endl;
            sleep(1);
        }
    }
    msg = "Connection successfully established with " + hostname;
    cout << msg << endl;

    vicon_client.EnableSegmentData();
    vicon_client.EnableMarkerData();
    vicon_client.EnableUnlabeledMarkerData();
    vicon_client.EnableMarkerRayData();
    vicon_client.EnableDeviceData();
    vicon_client.EnableDebugData();

    vicon_client.SetStreamMode(StreamMode::ClientPull);
    vicon_client.SetBufferSize(buffer_size);

    msg = "Initialization complete";
    cout << msg << endl;

    return true;
}

bool Communicator::disconnect()
{
    if (!vicon_client.IsConnected().Connected)
        return true;
    sleep(1);
    vicon_client.DisableSegmentData();
    vicon_client.DisableMarkerData();
    vicon_client.DisableUnlabeledMarkerData();
    vicon_client.DisableDeviceData();
    vicon_client.DisableCentroidData();
    string msg = "Disconnecting from " + hostname + "...";
    cout << msg << endl;
    vicon_client.Disconnect();
    msg = "Successfully disconnected";
    cout << msg << endl;
    if (!vicon_client.IsConnected().Connected)
        return true;
    return false;
}

void Communicator::get_frame()
{
    vicon_client.GetFrame();
    Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();

    unsigned int subject_count = vicon_client.GetSubjectCount().SubjectCount;

    map<string, Publisher>::iterator pub_it;

    for (unsigned int subject_index = 0; subject_index < subject_count; ++subject_index)
    {
        string subject_name = vicon_client.GetSubjectName(subject_index).SubjectName;
        unsigned int segment_count = vicon_client.GetSegmentCount(subject_name).SegmentCount;

        for (unsigned int segment_index = 0; segment_index < segment_count; ++segment_index)
        {
            string segment_name = vicon_client.GetSegmentName(subject_name, segment_index).SegmentName;
            PositionStruct current_position;
            Output_GetSegmentGlobalTranslation trans =
                vicon_client.GetSegmentGlobalTranslation(subject_name, segment_name);
            Output_GetSegmentGlobalRotationQuaternion rot =
                vicon_client.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
            
            for (size_t i = 0; i < 4; i++)
            {
                if (i < 3)
                    current_position.translation[i] = trans.Translation[i];
                current_position.rotation[i] = rot.Rotation[i];
            }
            current_position.segment_name = segment_name;
            current_position.subject_name = subject_name;
            current_position.translation_type = "Global";
            current_position.frame_number = frame_number.FrameNumber;

            boost::mutex::scoped_try_lock lock(mutex);

            if (lock.owns_lock())
            {
                pub_it = pub_map.find(subject_name + "/" + segment_name);
                if (pub_it != pub_map.end())
                {
                    Publisher & pub = pub_it->second;

                    if (pub.is_ready)
                    {
                        pub.publish(current_position);
                    }
                }
                else
                {
                    lock.unlock();
                    create_publisher(subject_name, segment_name);
                }
            }
        }
    }
}

void Communicator::create_publisher(const string subject_name, const string segment_name)
{
    boost::thread(&Communicator::create_publisher_thread, this, subject_name, segment_name);
}

void Communicator::create_publisher_thread(const string subject_name, const string segment_name)
{
    std::string topic_name = ns_name + "/" + subject_name + "/" + segment_name;
    std::string key = subject_name + "/" + segment_name;

    string msg = "Creating publisher for segment " + segment_name + " from subject " + subject_name;
    cout << msg << endl;

    geometry_msgs::msg::Quaternion quat_diff;

    string yaml_path = calibrations + "/" + subject_name + "/calibration.yaml";

    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config["x"] && config["y"] && config["z"] && config["w"]) {
            quat_diff.x = config["x"].as<double>();
            quat_diff.y = config["y"].as<double>();
            quat_diff.z = config["z"].as<double>();
            quat_diff.w = config["w"].as<double>();
        } else {
            std::cerr << "Quaternion data not found in YAML file, using unit quaternion: " << yaml_path << std::endl;
            quat_diff.x = 0.0;
            quat_diff.y = 0.0;
            quat_diff.z = 0.0;
            quat_diff.w = 1.0;
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Error reading YAML file, using unit quaternion: " << e.what() << std::endl;
        quat_diff.x = 0.0;
        quat_diff.y = 0.0;
        quat_diff.z = 0.0;
        quat_diff.w = 1.0;
    }
    geometry_msgs::msg::Quaternion quat_diff_ = normalize_quat(quat_diff);

    boost::mutex::scoped_lock lock(mutex);
    pub_map.insert(std::map<std::string, Publisher>::value_type(key, Publisher(topic_name, this, quat_diff_)));

    lock.unlock();
}

geometry_msgs::msg::Quaternion Communicator::normalize_quat(geometry_msgs::msg::Quaternion quat) {
        
    geometry_msgs::msg::Quaternion quat_norm;

    double magnitude = std::sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
    quat_norm.x = quat.x / magnitude;
    quat_norm.y = quat.y / magnitude;
    quat_norm.z = quat.z / magnitude;
    quat_norm.w = quat.w / magnitude;

    return quat_norm;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communicator>();
    node->connect();
    auto rate = rclcpp::Rate(node->fps_);

    while (rclcpp::ok()){
        node->get_frame();
        rate.sleep();
    }

    node->disconnect();
    rclcpp::shutdown();
    return 0;
}