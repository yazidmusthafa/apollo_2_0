from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the path to the URDF file
    urdf_file_name = 'apollo_2_0.urdf'
    urdf_file = os.path.join(
        get_package_share_directory('apollo_2_0'),  # Replace with your package name
        'urdf',
        urdf_file_name)

    # RViz config file
    rviz_config_file = os.path.join(
        get_package_share_directory('apollo_2_0'),  # Replace with your package name
        'rviz',
        'rviz_config.rviz')  # Optional: RViz config file path

    # Load the URDF into the parameter server
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    return LaunchDescription([
        # Publish the robot description to the parameter server
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        # Launch joint_state_publisher with GUI       
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        # Start RViz2 and load the configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
