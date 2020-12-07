from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('octomaps_comparator')

    stdout_logging_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_USE_STDOUT', '1')

    stdout_buffer_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Create the node:

    octomaps_comparator_node = Node(
        executable='octomaps_comparator_node',
        package='octomaps_comparator',
        name='octomaps_comparator_node',
        parameters=[
            {'octomap_kdtree_topic': 'octomap_kdtree'},
            {'octomap_erosion_topic': 'octomap_erosion'}
        ],
        remappings=[('antiguo', 'nuevo')],
        namespace=''
    )

    ld = LaunchDescription()

    ld.add_action(stdout_logging_envvar)
    ld.add_action(stdout_buffer_envvar)

    ld.add_action(octomaps_comparator_node)

    return ld