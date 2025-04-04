import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    gui_node = Node(
        package='p2-drone-formation-control-simulator',
        executable='gui_node'
    )

    config = os.path.join(
        get_package_share_directory('p2-drone-formation-control-simulator'),
        'parameters',
        'formation.yaml'
        )

    copter1_cmd_node = Node(
        package='p2-drone-formation-control-simulator',
        executable='copter1_cmd_node'
    )

    copter2_cmd_node = Node(
        package='p2-drone-formation-control-simulator',
        executable='copter2_cmd_node',
        parameters=[config]
    )
    
    copter3_cmd_node = Node(
        package='p2-drone-formation-control-simulator',
        executable='copter3_cmd_node',
        parameters=[config]
    )
  
    ld.add_action(gui_node)
    ld.add_action(copter1_cmd_node)
    ld.add_action(copter2_cmd_node)
    ld.add_action(copter3_cmd_node)
    return ld
