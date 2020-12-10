from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('octomaps_comparator')

    stdout_logging_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_USE_STDOUT', '1')

    stdout_buffer_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '0')

    # Create the node:

    octomaps_comparator_node = Node(
        executable='octomaps_comparator_node',
        package='octomaps_comparator',
        name='octomaps_comparator_node',
        parameters=[
            {'octomap_kdtree_topic': 'yolact_ros2_3d_node_octomaps/output_octomaps'},
            {'octomap_erosion_topic': 'yolact_ros2_3d/octomaps/dynamics/person'}
        ],
        #remappings=[
        #    ('/yolact_ros2_3d/octomaps/dynamics/person', '/octomap_erosion'),
        #    ('/yolact_ros2_3d_node_octomaps/output_octomaps', '/octomap_kdtree')
        #],
        namespace=''
    )

    ld = LaunchDescription()

    ld.add_action(stdout_logging_envvar)
    ld.add_action(stdout_buffer_envvar)

    ld.add_action(octomaps_comparator_node)

    return ld