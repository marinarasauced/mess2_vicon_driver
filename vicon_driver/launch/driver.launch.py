
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from os import path


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            "parameters",
            default_value=FindPackageShare(package="vicon_driver").find("vicon_driver")+"/launch/driver.yaml",
            description="path to the parameters file"
        ),

        Node(
            package="vicon_driver",
            executable="driver",
            parameters=[LaunchConfiguration("parameters")]
        )
    ])
