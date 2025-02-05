
"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value = 'True')

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', get_package_share_directory("apollo_2_0") + '/config',
            '-configuration_basename', 'apollo_mapping_2d.lua'],
        remappings = [
            ('echoes', 'horizontal_laser_2d')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': 0.05}],
        )

    rviz_config_dir = os.path.join(get_package_share_directory('apollo_2_0'),
                                   'rviz', 'cartographer.rviz')
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen')

    return LaunchDescription([
        use_sim_time,
        # Nodes
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])


# To save the map:   ros2 run nav2_map_server map_saver_cli -f ~/apollo_ws/src/apollo_2_0/config/map
