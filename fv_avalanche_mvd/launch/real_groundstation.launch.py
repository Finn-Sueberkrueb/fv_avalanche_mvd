from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():

    # load RVIZ2 with the configuration file
    rviz_config_path = os.path.join(
        os.path.dirname(__file__), '..','config', # Get the current directory of the launch file
        'simulation_conifg.rviz'
    )

    print(rviz_config_path)


    return LaunchDescription([

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]  # RViz config file path
        )

    ])
