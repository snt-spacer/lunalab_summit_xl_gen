#!/usr/bin/env -S ros2 launch
"""Launch of example for controlling Summit XL-GEN (LunaLab variant) in Ignition Gazebo with MoveIt2"""
"""To control the Summit XL mobile base, try `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/lunalab_summit_xl_gen/cmd_vel`"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from os import path
from typing import List


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")

    # List of included launch descriptions
    launch_descriptions = [
        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_ign_gazebo"),
                        "launch",
                        "ign_gazebo.launch.py",
                    ]
                )
            ),
            launch_arguments=[("ign_args", [world, " -v ", ign_verbosity])],
        ),
        # Launch move_group of MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lunalab_summit_xl_gen_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
            ],
        ),
        # Launch move_group of MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("lunalab_summit_xl_gen_ign"),
                        "launch",
                        "bridge.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("world_name", "moveit_follow_target_world"),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
            ],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # Bridge for position of the target
        Node(
            package="ros_ign_bridge",
            executable="parameter_bridge",
            output="log",
            arguments=[
                "/model/target/pose"
                + "@"
                + "geometry_msgs/msg/PoseStamped[ignition.msgs.Pose",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[("/model/target/pose", "/target_pose")],
        ),
        # Run the example script itself
        Node(
            package="lunalab_summit_xl_gen_ign",
            executable="ex_moveit_follow_target.py",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # # Run teleop twist keyboard for move base
        # Node(
        #     package="teleop_twist_keyboard",
        #     executable="teleop_twist_keyboard",
        #     output="screen",
        #     emulate_tty=True,
        #     arguments=["--ros-args", "--log-level", log_level],
        #     parameters=[{"use_sim_time": use_sim_time}],
        #     remappings=[("/cmd_vel", "/lunalab_summit_xl_gen/cmd_vel")],
        # ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # World for Ignition Gazebo
        DeclareLaunchArgument(
            "world",
            default_value=path.join(
                get_package_share_directory("lunalab_summit_xl_gen_ign"),
                "worlds",
                "moveit_follow_target.sdf",
            ),
            description="Name or filepath of world to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("lunalab_summit_xl_gen_ign"),
                "rviz",
                "ign_moveit.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="2",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
