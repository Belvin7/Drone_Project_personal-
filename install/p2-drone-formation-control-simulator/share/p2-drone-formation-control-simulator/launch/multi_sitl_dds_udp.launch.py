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
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_p2 = get_package_share_directory("p2-drone-formation-control-simulator")


    # Robot description 1.

    # Ensure `SDF_PATH` is populated as `sdformat_urdf`` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Load SDF file.
    sdf_file = os.path.join(
        pkg_p2, "models", "iris1", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()
        # print(robot_desc)

    # Publish /tf and /tf_static.
    robot_state_publisher1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ""},
        ],
    )

    iris_with_namespace = GroupAction(
        actions=[
            PushROSNamespace('iris1'),
            robot_state_publisher1,
        ]
    )

    # Robot description 2.

    # Ensure `SDF_PATH` is populated as `sdformat_urdf`` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Load SDF file.
    sdf_file = os.path.join(
        #pkg_ardupilot_gazebo, "models", "iris_with_gimbal", "model.sdf"
        pkg_p2, "models", "iris2", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()
        # print(robot_desc)

    # Publish /tf and /tf_static.
    robot_state_publisher2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ""},
        ],
    )

    iris2_with_namespace = GroupAction(
        actions=[
            PushROSNamespace('iris2'),
            robot_state_publisher2,
        ]
    )

    # Robot description 3.

    # Ensure `SDF_PATH` is populated as `sdformat_urdf`` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Load SDF file.
    sdf_file = os.path.join(
        #pkg_ardupilot_gazebo, "models", "iris_with_gimbal", "model.sdf"
        pkg_p2, "models", "iris3", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()
        # print(robot_desc)

    # Publish /tf and /tf_static.
    robot_state_publisher3 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ""},
        ],
    )

    iris3_with_namespace = GroupAction(
        actions=[
            PushROSNamespace('iris3'),
            robot_state_publisher3,
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
        gz_sim_server,
        gz_sim_gui,
        iris_with_namespace,
        iris2_with_namespace,
        iris3_with_namespace,
        #rviz,
        bridge,
        RegisterEventHandler(
            OnProcessStart(
                target_action=bridge,
                on_start=[
                    topic_tools_tf
                ]
            )
        ),
    ])

