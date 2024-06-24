import os


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command

import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    pkg_path = os.path.join(get_package_share_directory('recognizer'))
    xacro_file = os.path.join(pkg_path,'description', 'robot_core.xacro')

    robot_description_config = Command(['xacro ', xacro_file])

    state_publisher_params = {'robot_description': robot_description_config}

    recognizer_state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[state_publisher_params]
        )


    return LaunchDescription([

        recognizer_state_publisher
        
    ])