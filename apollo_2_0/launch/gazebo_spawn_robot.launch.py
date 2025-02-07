import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

import xacro

def generate_launch_description():

    use_rviz_args = DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz')
    use_sim_time_args = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Sim time')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz')

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
    params = {'use_sim_time': use_sim_time, 'robot_description': doc.toxml()}

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
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_file]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'apollo_2_0',
                                '-z', '0.05',
                                '-x', '0.0',
                                '-y', '9.5',
                                '-Y', '-1.5708'
                                ],
                    output='screen')

    # Run the node
    return LaunchDescription([
        use_rviz_args,
        use_sim_time_args,
        node_robot_state_publisher,
        spawn_entity,
        rviz2,

    ])

