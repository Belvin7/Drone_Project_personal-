from pathlib import Path


import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushROSNamespace,SetRemap
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    """Generate a launch description to bring up multiple ArduPilot SITL instances with MAVProxy and Map visualization."""

    """Generate a launch description for a iris quadcopter."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_p2 = get_package_share_directory("p2-drone-formation-control-simulator")

    # Iris.
    iris = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('p2-drone-formation-control-simulator'), 'launch', 'iris_member1.launch.py'])
        )
    )

    iris_with_namespace = GroupAction(
        actions=[
            PushROSNamespace('iris1'),
            #SetRemap(src='/ap/', dst='/iris1/'),
            iris,
        ]
    )

    # Iris2.
    iris2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('p2-drone-formation-control-simulator'), 'launch', 'iris_member2.launch.py'])
        )
    )

    iris2_with_namespace = GroupAction(
        actions=[
            PushROSNamespace('iris2'),
            iris2,
        ]
    )

    # Gazebo.
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
                       f'{Path(pkg_p2) / "world" / "iris_runway.sdf"}'
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )


    # Define launch descriptions for multiple SITL instances
    sitl_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ardupilot_sitl'), 'launch', 'sitl.launch.py'])
        ),
        launch_arguments={
            'model': 'quad',
            'speedup': '1',
            'synthetic_clock': 'True',
            'wipe': 'False',
            'instance': '0',
            'serial1': 'uart:/dev/ttyS1',
            'sim_address': '127.0.0.1',
            'master': 'tcp:127.0.0.1:5760',
            'sitl': '127.0.0.1:5501',
            'sysid': '1'

        }.items()
    )

    sitl_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ardupilot_sitl'), 'launch', 'sitl.launch.py'])
        ),
        launch_arguments={
            'model': 'quad',
            'speedup': '1',
            'synthetic_clock': 'True',
            'wipe': 'False',
            'instance': '1',
            'serial1': 'uart:/dev/ttyS2',
            'sim_address': '127.0.0.1',
            'master': 'tcp:127.0.0.1:5770',
            'sitl': '127.0.0.1:5511',
            'sysid': '2'
        }.items()
    )

    sitl_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ardupilot_sitl'), 'launch', 'sitl.launch.py'])
        ),
        launch_arguments={
            'model': 'quad',
            'speedup': '1',
            'synthetic_clock': 'True',
            'wipe': 'False',
            'instance': '2',
            'serial1': 'uart:/dev/ttyS3',
            'sim_address': '127.0.0.1',
            'master': 'tcp:127.0.0.1:5780',
            'sitl': '127.0.0.1:5521',
            'sysid': '3'
        }.items()
    )

    # MAVProxy for first drone
    mavproxy_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ardupilot_sitl'), 'launch', 'mavproxy.launch.py'])
        ),
        launch_arguments={
            'map': 'False',
            'console': 'True',
            'master': 'tcp:127.0.0.1:5760',
            'sitl': '127.0.0.1:5501'
        }.items()
    )

    # MAVProxy for second drone
    mavproxy_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ardupilot_sitl'), 'launch', 'mavproxy.launch.py'])
        ),
        launch_arguments={
            'map': 'False',
            'console': 'True',
            'master': 'tcp:127.0.0.1:5770',
            'sitl': '127.0.0.1:5511'
        }.items()
    )

    # RViz for map visualization for first drone
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "iris.rviz"}'],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Bridge.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", "iris_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Relay - use instead of transform when Gazebo is only publishing odom -> base_link
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=[
            "/gz/tf",
            "/tf",
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Open RViz.",
        ),
        DeclareLaunchArgument(
            "use_gz_tf", default_value="true", description="Use Gazebo TF."
        ),
        #gz_sim_server,
        #gz_sim_gui,
        #iris_with_namespace,
        #iris2_with_namespace,
        sitl_1,
        sitl_2,
        mavproxy_1,
        mavproxy_2,
        #rviz,
        #bridge,
        #RegisterEventHandler(
        #    OnProcessStart(
        #        target_action=bridge,
        #        on_start=[
        #            topic_tools_tf
        #        ]
        #    )
        #),
    ])

