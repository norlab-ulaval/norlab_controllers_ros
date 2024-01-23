import os
import yaml
import errno

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def launch_controller_node(context, *args, **kwargs):

    arg_robot = context.perform_substitution(LaunchConfiguration('robot'))
    arg_terrain = context.perform_substitution(LaunchConfiguration('terrain'))
    arg_controller = context.perform_substitution(LaunchConfiguration('controller'))

    # Get full path to config file
    controller_config_path = os.path.join(
        get_package_share_directory('norlab_controllers_wrapper'),
        'params',
        arg_robot,
        arg_terrain,
        arg_controller + '.yaml'
    )
    rotation_config_path = os.path.join(
        get_package_share_directory('norlab_controllers_wrapper'),
        'params',
        arg_robot,
        arg_terrain,
        'rotation-p.yaml'
    )

    # Check if config file exists
    if not os.path.exists(controller_config_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), controller_config_path)
        
    # Controller Node
    controller_wrapper_node = Node(
        package='norlab_controllers_wrapper',
        executable='controller_node.py',
        name="controller_node",
        output='screen',
        parameters=[
            {"controller_config": controller_config_path},
            {"rotation_controller_config": rotation_config_path}
        ],
        remappings=[
            ("odom_in", "icp_odom"),
            ("cmd_vel_out", "nav_vel")
        ]
    )

    return [controller_wrapper_node]


def generate_launch_description():

    params_path = os.path.join(get_package_share_directory('norlab_controllers_wrapper'), 'params') 
    general_config_path = os.path.join(params_path, 'general.yaml') 
    config = yaml.safe_load(open(general_config_path, 'r'))
    
    # Declare command line arguments
    robot_argument = DeclareLaunchArgument(
        'robot',
        default_value=config['robot'],
        description='Robot type: [husky, marmotte, warthog_wheels, warthog_tracks]'
    )
    terrain_argument = DeclareLaunchArgument(
        'terrain',
        default_value=config['terrain'],
        description='Terrain type: [grass, snow]'
    )
    controller_argument = DeclareLaunchArgument(
        'controller',
        default_value=config['controller'],
        description='Controller type: [differential-orthexp, ideal-diff-drive-mpc]'
    )
    
    return LaunchDescription([
        robot_argument,
        terrain_argument,
        controller_argument,
        OpaqueFunction(function=launch_controller_node)
    ])