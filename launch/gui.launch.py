from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    gui_node = Node(
        package='p2-drone-formation-control-simulator',
        executable='gui_node'
    )
    
    # only start for one drone
    #cmd_node = Node(
    #    package='p2-drone-formation-control-simulator',
    #    executable='cmd_node'
    #)

    copter1_cmd_node = Node(
        package='p2-drone-formation-control-simulator',
        executable='copter1_cmd_node'
    )

    copter2_cmd_node = Node(
        package='p2-drone-formation-control-simulator',
        executable='copter2_cmd_node'
    )
    
    copter3_cmd_node = Node(
        package='p2-drone-formation-control-simulator',
        executable='copter3_cmd_node'
    )
  
    ld.add_action(gui_node)
    #ld.add_action(cmd_node)
    ld.add_action(copter1_cmd_node)
    ld.add_action(copter2_cmd_node)
    ld.add_action(copter3_cmd_node)
    return ld
