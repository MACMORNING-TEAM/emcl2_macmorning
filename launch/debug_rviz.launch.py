import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    local_rviz = os.path.join(
        os.getenv('PWD', ''),    # 워크스페이스 루트 경로
        'src',
        'emcl2_ros2',            # 실제 패키지 폴더 이름
        'rviz',
        'local.rviz'
    )

    # 커스텀 RViz 노드
    custom_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_local',
        output='screen',
        arguments=['-d', local_rviz]
    )

    return LaunchDescription([
        custom_rviz_node
    ])
