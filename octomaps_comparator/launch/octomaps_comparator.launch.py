from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('octomaps_comparator')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Create the node:

    octomaps_comparator_node = Node(
        node_executable='octomaps_comparator_node',
        package='octomaps_comparator',
        node_name='octomaps_comparator_node',
        parameters=[
            {'octomap_kdtree_topic': 'octomap_kdtree'},
            {'octomap_erosion_topic': 'octomap_erosion'}
        ],
        remappings=[('antiguo', 'nuevo')],
        node_namespace=''
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(octomaps_comparator_node)

    return ld