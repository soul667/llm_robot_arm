from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('llm_proxy')
    
    # Default params file path
    default_params_file = os.path.join(pkg_share, 'config', 'params.yml')
    
    # Declare the params file argument
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )
    
    # Create the node with namespace
    llm_proxy_node = Node(
        package='llm_proxy',
        executable='llm_proxy_node',
        name='llm_proxy_node',
        namespace='',  # Empty namespace to use top-level
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],  # Enable info level logging
    )
    
    # Return the launch description
    return LaunchDescription([
        params_file_arg,
        llm_proxy_node
    ])
