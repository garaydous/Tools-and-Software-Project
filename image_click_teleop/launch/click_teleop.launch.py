import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    DeclareLaunchArgument, 
    TimerAction,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.17.0.2',
        description='IP address of the UR5e robot or URSim'
    )

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'launch_rviz': 'true',
        }.items(),
    )

    switch_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--activate', 'scaled_joint_trajectory_controller',
            '--deactivate', 'joint_trajectory_controller'
        ],
        shell=False,
        output='screen'
    )

    # Camera node
    camera_node = Node(
        package='image_click_teleop',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen'
    )

    teleop_node = Node(
        package='image_click_teleop',
        executable='click_teleop',
        name='click_teleop_node',
        output='screen'
    )

    # Delay controller switch to ensure controllers are loaded
    delayed_switch = TimerAction(
        period=5.0,
        actions=[switch_controllers]
    )

    delayed_camera = TimerAction(
        period=8.0,
        actions=[camera_node]
    )

    delayed_teleop = TimerAction(
        period=9.0,
        actions=[teleop_node]
    )

    return LaunchDescription([
        robot_ip_arg,
        ur_launch,
        delayed_switch,
        delayed_camera,
        delayed_teleop,
    ])

