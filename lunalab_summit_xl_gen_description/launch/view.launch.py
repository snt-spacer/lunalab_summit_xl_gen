#!/usr/bin/env -S ros2 launch
"""Visualisation of URDF model for lunalab_summit_xl_gen in RViz2"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from typing import List


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_position_margin = LaunchConfiguration("safety_position_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    high_quality_mesh = LaunchConfiguration("high_quality_mesh")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Extract URDF from description file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package),
                                  description_filepath]),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_position_margin:=",
            safety_position_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "high_quality_mesh:=",
            high_quality_mesh,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # List of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="log",
            parameters=[robot_description,
                        {"use_sim_time": use_sim_time}],
        ),
        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["--display-config", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # joint_state_publisher_gui
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="log",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Location of xacro/URDF to visualise
        DeclareLaunchArgument(
            "description_package",
            default_value="lunalab_summit_xl_gen_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=PathJoinSubstitution(["urdf",
                                                "lunalab_summit_xl_gen.urdf.xacro"]),
            description="Path to xacro or URDF description of the robot, relative to share of `description_package`.",
        ),

        # Naming of the robot
        DeclareLaunchArgument(
            "name",
            default_value="lunalab_summit_xl_gen",
            description="Name of the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for all robot entities. If modified, then joint names in the configuration of controllers must also be updated.",
        ),

        # Safety controller
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Flag to enable safety limits controllers on manipulator joints.",
        ),
        DeclareLaunchArgument(
            "safety_position_margin",
            default_value="0.15",
            description="Lower and upper margin for position limits of all safety controllers.",
        ),
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="Parametric k-position factor of all safety controllers.",
        ),

        # Geometry
        DeclareLaunchArgument(
            "high_quality_mesh",
            default_value="true",
            description="Flag to select the high or low quality model.",
        ),

        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([FindPackageShare("lunalab_summit_xl_gen_description"),
                                                "rviz",
                                                "view.rviz"]),
            description="Path to configuration for RViz2."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock."
        ),
    ]
