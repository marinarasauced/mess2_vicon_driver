from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    hostname = '192.168.0.45'
    buffer_size = 200
    topic_namespace = 'vicon'
    fps = 40.0

    return LaunchDescription([Node(
            package='vicon_receiver', executable='vicon_client', output='screen',
            parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace, 'fps': fps}]
        )])
