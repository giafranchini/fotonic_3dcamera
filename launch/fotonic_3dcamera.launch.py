from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

container = ComposableNodeContainer(
    name='container_fotonic',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
        ComposableNode(
            package='fotonic_3dcamera',
            plugin='fotonic::Camera3D',
            name='fotonic_3dcamera',
            parameters=[os.path.join(get_package_share_directory("fotonic_3dcamera"), 'params', 'params.yaml')],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ],
    output={
            "stdout": "screen",
            "stderr": "screen",
    },
)

def generate_launch_description():
    return LaunchDescription([container])
