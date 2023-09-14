from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():


    return LaunchDescription([
        
        DeclareLaunchArgument(
            'ssid',
            default_value='testfvs',  # Enter your SSID here
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

        Node(
            package='fv_avalanche_mvd',
            executable='search_sequence',
            output='screen',
        ),
        Node(
            package='fv_avalanche_mvd',
            executable='transponder_localization',
            output='screen',
        ),

    ])
