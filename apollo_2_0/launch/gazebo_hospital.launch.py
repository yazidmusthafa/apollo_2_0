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

    world = os.path.join(
        get_package_share_directory('apollo_2_0'),
        'worlds',
        'empty_hospital.world'
    )

    # world = os.path.join(
    #     get_package_share_directory('apollo_2_0'),
    #     'worlds',
    #     'hospital.world'
    # )

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
   
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Run the node
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,

    ])

