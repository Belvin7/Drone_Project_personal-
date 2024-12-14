from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    gui_node = Node(
        package='p2-drone-formation-control-simulator',
        executable='gui_node'
    )

    cmd_node = Node(
        package='p2-drone-formation-control-simulator',
        executable='cmd_node'
    )
    
    ld.add_action(gui_node)
    ld.add_action(cmd_node)
    return ld
