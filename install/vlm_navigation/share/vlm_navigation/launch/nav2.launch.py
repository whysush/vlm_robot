"""Nav2 + AMCL bringup, localizing against a saved map.

Prereq: gazebo bringup is running (provides /scan, /odom, odom->base TF). Starts
map_server, amcl, planner, controller, behaviors, bt_navigator, velocity_smoother
and two lifecycle managers. AMCL is seeded via nav2_params.yaml so the robot is
localized immediately.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    nav_pkg = get_package_share_directory('vlm_navigation')
    desc_pkg = get_package_share_directory('vlm_description')

    params_file = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    default_map = os.path.join(nav_pkg, 'maps', 'map.yaml')
    rviz_cfg = os.path.join(desc_pkg, 'rviz', 'robot.rviz')

    # Force FastDDS for the whole stack; the default RMW (CycloneDDS) breaks Nav2
    # on loopback. Every shell that talks to this stack must export the same.
    set_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')
    autostart = LaunchConfiguration('autostart')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map,
            description='Full path to map yaml.'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    lifecycle_localization = ['map_server', 'amcl']
    lifecycle_navigation = ['controller_server', 'smoother_server',
                            'planner_server', 'behavior_server',
                            'bt_navigator', 'velocity_smoother']

    bringup = GroupAction([
        SetParameter('use_sim_time', use_sim_time),

        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[params_file, {'yaml_filename': map_yaml}]),

        Node(package='nav2_amcl', executable='amcl',
             name='amcl', output='screen',
             parameters=[params_file]),

        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[params_file],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),

        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen',
             parameters=[params_file]),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[params_file]),

        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[params_file]),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[params_file]),

        # velocity_smoother emits the final /cmd_vel the DiffDrive bridge reads.
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[params_file],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),

        # Long bond/service timeouts ride out CPU saturation during bringup.
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': autostart,
                          'node_names': lifecycle_localization,
                          'bond_timeout': 30.0,
                          'attempt_respawn_reconnection': True,
                          'bond_respawn_max_duration': 30.0}]),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': autostart,
                          'node_names': lifecycle_navigation,
                          'bond_timeout': 30.0,
                          'attempt_respawn_reconnection': True,
                          'bond_respawn_max_duration': 30.0}]),
    ])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([set_rmw] + declare + [bringup, rviz])
