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
    
    use_sim_time_args = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Sim time')
    use_sim_time = LaunchConfiguration('use_sim_time')

    apollo_pkg_path = get_package_share_directory('apollo_2_0')

    config_dir = os.path.join(apollo_pkg_path,'config')
    map_file = os.path.join(config_dir,'empty_hospital_map.yaml')
    param_file = os.path.join(config_dir,'apollo_nav2_params.yaml')
    rviz_config_dir = os.path.join(config_dir,'navigation.rviz')


    gazebo_hospital_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(apollo_pkg_path, 'launch', 'gazebo_hospital.launch.py')
        )
    )

    spawn_apollo_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(apollo_pkg_path, 'launch', 'gazebo_spawn_robot.launch.py')
        ),
        launch_arguments={
            'use_rviz':'false'}.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
            launch_arguments={
            'map':map_file,
            'params_file': param_file}.items(),
        )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
        )
    
    set_initial_robot_pose_node = Node(
        package='apollo_2_0',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
        )

    # Run the node
    return LaunchDescription([
        use_sim_time_args,
        gazebo_hospital_world,
        spawn_apollo_robot,
        nav2_launch,
        rviz_node,
        set_initial_robot_pose_node,
    ])

