"""slam_toolbox online-async mapping.

Prereq: gazebo bringup is already running (vlm_description gazebo.launch.py), so
/scan, /odom and the odom->base_footprint TF exist. This node produces the map
and the map->odom transform.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('vlm_navigation')
    desc_pkg = get_package_share_directory('vlm_description')

    slam_params = os.path.join(nav_pkg, 'config', 'slam_toolbox.yaml')
    rviz_cfg = os.path.join(desc_pkg, 'rviz', 'robot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(declare + [slam, rviz])
