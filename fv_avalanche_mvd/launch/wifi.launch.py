import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'ssid',
            default_value='YOUR_SSID',  # Enter your SSID here
            description='SSID of the WiFi network to monitor'
        ),
        #LogInfo(msg=LaunchConfiguration('ssid')),
        
        Node(
            package='fv_avalanche_mvd',
            #executable='read_wifi_pyWiFi',
            executable='read_wifi',
            output='screen',
            parameters=[{'ssid': LaunchConfiguration('ssid')}]
            ),
    ])