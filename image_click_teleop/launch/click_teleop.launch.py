import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    set_tb3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )

    tb3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            )
        )
    )

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

    return LaunchDescription([
        set_tb3_model,
        tb3_gazebo_launch,
        camera_node,
        teleop_node,
    ])

