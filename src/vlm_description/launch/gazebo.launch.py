"""Gazebo bringup: sim world, robot_state_publisher, robot spawn, ros_gz
bridges for sensors, and optional RViz.

render_engine selects ogre2 (default) or ogre. software_render forces Mesa
software GL (llvmpipe) for sensor rendering when ogre2 can't init on an iGPU.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, AppendEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                   Command, FindExecutable)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('vlm_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'world.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')
    rviz_cfg   = os.path.join(pkg, 'rviz', 'robot.rviz')

    use_sim_time   = LaunchConfiguration('use_sim_time')
    render_engine  = LaunchConfiguration('render_engine')
    software_render = LaunchConfiguration('software_render')
    headless       = LaunchConfiguration('headless')
    use_rviz       = LaunchConfiguration('rviz')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre2',
            description='gz render engine for the GUI/server: ogre2 or ogre'),
        DeclareLaunchArgument('software_render', default_value='false',
            description='Force Mesa software GL (llvmpipe) for sensor rendering '
                        'when ogre2 fails. true/false'),
        DeclareLaunchArgument('headless', default_value='false',
            description='Run gz server only (no GUI). Sensors still render.'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    set_env = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(pkg, 'worlds')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
    ]
    set_env.append(
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE',
                               LaunchConfiguration('software_render')))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [world_file,
                        ' -r -v 3 --render-engine ', render_engine]
        }.items(),
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.12'],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': use_sim_time}],
    )

    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge', output='screen',
        arguments=['/rgbd/image', '/rgbd/depth_image'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(declare + set_env + [
        gz_sim, rsp, spawn, bridge, image_bridge, rviz,
    ])
