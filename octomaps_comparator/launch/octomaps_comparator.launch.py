from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('octomaps_comparator')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Create the node

    octomaps_comparator_node = Node(
        node_executable='octomaps_comparator_node',
        package='octomaps_comparator',
        node_name='octomaps_comparator_node',
        parameters=['kdtree_octomap', 'erosion_octomap'],
        remappings=[('antiguo', 'nuevo')]
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    return ld