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
            package='fv_avalanche_mvd',
            executable='signal_simulation',
            output='screen',
            parameters=[
                {'transponder_position_x': 3.0},
                {'transponder_position_y': 1.0},
                {'transponder_position_z': 0.0},
                {'transponder_position_noise': 0.1},
            ],
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
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]  # RViz config file path
        )

    ])
