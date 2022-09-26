import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('bump_and_go'),
        'config',
        'config.yaml'
        )
        
    node=Node(
        package = 'bump_and_go',
        name = 'bump_and_go_node',
        executable = 'bump_and_go_node',
        parameters = [config]
    )

    ld.add_action(node)
    return ld