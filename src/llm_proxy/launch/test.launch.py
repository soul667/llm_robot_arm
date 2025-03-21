from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('llm_proxy')
    
    # Default params file path
    default_params_file = os.path.join(pkg_share, 'config', 'params.yml')
    
    # Launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use'
    )
    
    run_test_arg = DeclareLaunchArgument(
        'run_test',
        default_value='true',
        description='Whether to run the test client node'
    )
    
    # LLM Proxy node
    llm_proxy_node = Node(
        package='llm_proxy',
        executable='llm_proxy_node',
        name='llm_proxy_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Test client node (conditional)
    test_client_node = Node(
        package='llm_proxy',
        executable='test_client_node',
        name='test_client_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        condition=IfCondition(LaunchConfiguration('run_test'))
    )
    
    # Return the launch description
    return LaunchDescription([
        params_file_arg,
        run_test_arg,
        llm_proxy_node,
        test_client_node
    ])
