"""One-shot bringup: Gazebo + Nav2 (+ optional RViz).

Includes the gazebo and nav2 launch files; Nav2 starts after a short delay so
/scan and the odom TF exist before AMCL and the costmaps come up. Run the
object-goal node separately once this is active:

  ros2 run vlm_navigation object_goal_nav --ros-args -p target:="red box"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    desc_pkg = get_package_share_directory('vlm_description')
    nav_pkg = get_package_share_directory('vlm_navigation')

    use_rviz = LaunchConfiguration('rviz')
    nav2_delay = LaunchConfiguration('nav2_delay')

    declare = [
        DeclareLaunchArgument('rviz', default_value='true',
            description='Open RViz with the Nav2 view.'),
        DeclareLaunchArgument('nav2_delay', default_value='10.0',
            description='Seconds to wait after Gazebo before starting Nav2.'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'rviz': 'false'}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'nav2.launch.py')),
        launch_arguments={'rviz': use_rviz}.items(),
    )

    nav2_delayed = TimerAction(period=nav2_delay, actions=[nav2])

    return LaunchDescription(declare + [gazebo, nav2_delayed])
