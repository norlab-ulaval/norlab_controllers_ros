import os
import errno

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare command line arguments
    controller_argument = DeclareLaunchArgument(
        'controller',
        default_value='ideal-articulated-drive-mpc',
        description='Controller type: [differential-orthexp, ideal-diff-drive-mpc, ideal-articulated-drive-mpc]',
    )
    namespace_argument = DeclareLaunchArgument(
        'controller_ns',
        default_value="controller",
    )
    
    return LaunchDescription([
        controller_argument,
        namespace_argument,
        OpaqueFunction(function=launch_controller_node)
    ])


def launch_controller_node(context, *args, **kwargs):

    config_path = os.path.join(get_package_share_directory('norlab_controllers_wrapper'), 'config') 
    arg_controller = context.perform_substitution(LaunchConfiguration('controller'))
    namespace = context.perform_substitution(LaunchConfiguration('controller_ns'))

    # Get full path to config file
    controller_config_path = os.path.join(
        config_path,
        arg_controller + '.yaml'
    )

    # Check if config file exists
    if not os.path.exists(controller_config_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), controller_config_path)
        
    # Controller Node
    controller_wrapper_node = Node(
        package='norlab_controllers_wrapper',
        executable='controller_node.py',
        name="controller_node",
        namespace=namespace,
        output='screen',
        parameters=[
            {"controller_config": controller_config_path},
            {"map_frame": "map"},
            {"robot_frame": "base_link"},
            {"follow_path_topic": "follow_path"},
        ]
    )

    return [controller_wrapper_node]