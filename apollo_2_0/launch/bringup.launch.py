import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():

    urdf_file = os.path.join(get_package_share_directory(package_name="mini_ugv_asm"), "urdf", "mini_ugv_asm.urdf")
    urdf = open(urdf_file).read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {'robot_description': urdf},
            {"use_tf_static": False},
            {"publish_frequencey": 200.0},
            {"ignore_timestamp": True},
            {"use_sim_sim": True}
        ]
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_node)

    return ld