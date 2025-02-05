import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro

def generate_launch_description():
   
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']), 
    )

    # RViz config file
    rviz_config_file = os.path.join(
        get_package_share_directory('apollo_2_0'),  # Replace with your package name
        'rviz',
        'rviz_config.rviz')  # Optional: RViz config file path
        
    package_path = os.path.join(
        get_package_share_directory('apollo_2_0'))
    
    # Use xacro to process the file
    urdf_file = os.path.join(package_path,'urdf','apollo_2_0_gazebo.urdf.xacro')

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'apollo_2_0',
                                '-z', '1.0',
                                '-x', '0.0',
                                '-y', '0.0',
                                ],
                    output='screen')

    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        rviz2
    ])