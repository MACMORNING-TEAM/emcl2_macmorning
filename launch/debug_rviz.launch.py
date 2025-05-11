import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():

    # 이 런치 파일이 src/emcl2_ros2/launch 디렉터리에 있다고 가정
    # Path(__file__) → .../src/emcl2_ros2/launch/your_launch.py
    pkg_root = Path(__file__).resolve().parents[1]
    # → .../src/emcl2_ros2

    local_rviz = pkg_root / 'rviz' / 'local.rviz'

    custom_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_local',
        output='screen',
        arguments=['-d', str(local_rviz)],
    )

    return LaunchDescription([
        custom_rviz_node
    ])
