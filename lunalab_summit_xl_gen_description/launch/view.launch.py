#!/usr/bin/env -S ros2 launch
"""Visualisation of URDF model for lunalab_summit_xl_gen in RViz2"""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    external_devices = LaunchConfiguration("external_devices")
    realsense_d435 = LaunchConfiguration("realsense_d435")
    realsense_d455 = LaunchConfiguration("realsense_d455")
    realsense_l515 = LaunchConfiguration("realsense_l515")
    lidar = LaunchConfiguration("lidar")
    collision_chassis = LaunchConfiguration("collision_chassis")
    collision_wheels = LaunchConfiguration("collision_wheels")
    collision_arm = LaunchConfiguration("collision_arm")
    collision_gripper = LaunchConfiguration("collision_gripper")
    collision_external_devices = LaunchConfiguration("collision_external_devices")
    collision_realsense_d435 = LaunchConfiguration("collision_realsense_d435")
    collision_realsense_d455 = LaunchConfiguration("collision_realsense_d455")
    collision_realsense_l515 = LaunchConfiguration("collision_realsense_l515")
    collision_lidar = LaunchConfiguration("collision_lidar")
    high_quality_mesh = LaunchConfiguration("high_quality_mesh")
    mimic_gripper_joints = LaunchConfiguration("mimic_gripper_joints")
    ros2_control = LaunchConfiguration("ros2_control")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    ros2_control_command_interface = LaunchConfiguration(
        "ros2_control_command_interface"
    )
    gazebo_preserve_fixed_joint = LaunchConfiguration("gazebo_preserve_fixed_joint")
    gazebo_self_collide = LaunchConfiguration("gazebo_self_collide")
    gazebo_self_collide_fingers = LaunchConfiguration("gazebo_self_collide_fingers")
    gazebo_diff_drive = LaunchConfiguration("gazebo_diff_drive")
    gazebo_joint_trajectory_controller = LaunchConfiguration(
        "gazebo_joint_trajectory_controller"
    )
    gazebo_joint_state_publisher = LaunchConfiguration("gazebo_joint_state_publisher")
    gazebo_pose_publisher = LaunchConfiguration("gazebo_pose_publisher")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # Extract URDF from description file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_filepath]
            ),
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
            "external_devices:=",
            external_devices,
            " ",
            "realsense_d435:=",
            realsense_d435,
            " ",
            "realsense_d455:=",
            realsense_d455,
            " ",
            "realsense_l515:=",
            realsense_l515,
            " ",
            "lidar:=",
            lidar,
            " ",
            "collision_chassis:=",
            collision_chassis,
            " ",
            "collision_wheels:=",
            collision_wheels,
            " ",
            "collision_arm:=",
            collision_arm,
            " ",
            "collision_gripper:=",
            collision_gripper,
            " ",
            "collision_external_devices:=",
            collision_external_devices,
            " ",
            "collision_realsense_d435:=",
            collision_realsense_d435,
            " ",
            "collision_realsense_d455:=",
            collision_realsense_d455,
            " ",
            "collision_realsense_l515:=",
            collision_realsense_l515,
            " ",
            "collision_lidar:=",
            collision_lidar,
            " ",
            "high_quality_mesh:=",
            high_quality_mesh,
            " ",
            "mimic_gripper_joints:=",
            mimic_gripper_joints,
            " ",
            "ros2_control:=",
            ros2_control,
            " ",
            "ros2_control_plugin:=",
            ros2_control_plugin,
            " ",
            "ros2_control_command_interface:=",
            ros2_control_command_interface,
            " ",
            "gazebo_preserve_fixed_joint:=",
            gazebo_preserve_fixed_joint,
            " ",
            "gazebo_self_collide:=",
            gazebo_self_collide,
            " ",
            "gazebo_self_collide_fingers:=",
            gazebo_self_collide_fingers,
            " ",
            "gazebo_diff_drive:=",
            gazebo_diff_drive,
            " ",
            "gazebo_joint_trajectory_controller:=",
            gazebo_joint_trajectory_controller,
            " ",
            "gazebo_joint_state_publisher:=",
            gazebo_joint_state_publisher,
            " ",
            "gazebo_pose_publisher:=",
            gazebo_pose_publisher,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # List of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),
        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=[
                "--display-config",
                rviz_config,
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # joint_state_publisher_gui
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
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
            default_value=path.join("urdf", "lunalab_summit_xl_gen.urdf.xacro"),
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
            default_value="robot_",
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
        # Enables of sensors and miscellaneous parts
        DeclareLaunchArgument(
            "external_devices",
            default_value="false",
            description="Flag to enable external devices (mesh).",
        ),
        DeclareLaunchArgument(
            "realsense_d435",
            default_value="false",
            description="Flag to enable RealSense D435.",
        ),
        DeclareLaunchArgument(
            "realsense_d455",
            default_value="false",
            description="Flag to enable RealSense D455.",
        ),
        DeclareLaunchArgument(
            "realsense_l515",
            default_value="false",
            description="Flag to enable RealSense L515.",
        ),
        DeclareLaunchArgument(
            "lidar",
            default_value="false",
            description="Flag to enable LiDAR.",
        ),
        # Collision geometry
        DeclareLaunchArgument(
            "collision_chassis",
            default_value="true",
            description="Flag to enable collision geometry for the chassis of Summit XL.",
        ),
        DeclareLaunchArgument(
            "collision_wheels",
            default_value="true",
            description="Flag to enable collision geometry for the wheels of Summit XL.",
        ),
        DeclareLaunchArgument(
            "collision_arm",
            default_value="true",
            description="Flag to enable collision geometry for manipulator's arm.",
        ),
        DeclareLaunchArgument(
            "collision_gripper",
            default_value="true",
            description="Flag to enable collision geometry for manipulator's gripper (hand and fingers).",
        ),
        DeclareLaunchArgument(
            "collision_external_devices",
            default_value="true",
            description="Flag to enable collision geometry for external devices.",
        ),
        DeclareLaunchArgument(
            "collision_realsense_d435",
            default_value="true",
            description="Flag to enable collision geometry for RealSense D435.",
        ),
        DeclareLaunchArgument(
            "collision_realsense_d455",
            default_value="true",
            description="Flag to enable collision geometry for RealSense D455.",
        ),
        DeclareLaunchArgument(
            "collision_realsense_l515",
            default_value="true",
            description="Flag to enable collision geometry for RealSense L515.",
        ),
        DeclareLaunchArgument(
            "collision_lidar",
            default_value="true",
            description="Flag to enable collision geometry for LiDAR.",
        ),
        # Geometry
        DeclareLaunchArgument(
            "high_quality_mesh",
            default_value="true",
            description="Flag to select the high or low quality model.",
        ),
        # Gripper
        DeclareLaunchArgument(
            "mimic_gripper_joints",
            default_value="false",
            description="Flag to mimic joints of the gripper.",
        ),
        # ROS 2 control
        DeclareLaunchArgument(
            "ros2_control",
            default_value="true",
            description="Flag to enable ros2 controllers for manipulator.",
        ),
        DeclareLaunchArgument(
            "ros2_control_plugin",
            default_value="ign",
            description="The ros2_control plugin that should be loaded for the manipulator ('fake', 'ign', 'real' or custom).",
        ),
        DeclareLaunchArgument(
            "ros2_control_command_interface",
            default_value="effort",
            description="The output control command interface provided by ros2_control ('position', 'velocity' or 'effort').",
        ),
        # Gazebo
        DeclareLaunchArgument(
            "gazebo_preserve_fixed_joint",
            default_value="false",
            description="Flag to preserve fixed joints and prevent lumping when generating SDF for Gazebo.",
        ),
        DeclareLaunchArgument(
            "gazebo_self_collide",
            default_value="false",
            description="Flag to enable self-collision of all robot links when generating SDF for Gazebo.",
        ),
        DeclareLaunchArgument(
            "gazebo_self_collide_fingers",
            default_value="true",
            description="Flag to enable self-collision of robot between fingers (finger tips) when generating SDF for Gazebo.",
        ),
        DeclareLaunchArgument(
            "gazebo_diff_drive",
            default_value="true",
            description="Flag to enable DiffDrive Gazebo plugin for Summit XL.",
        ),
        DeclareLaunchArgument(
            "gazebo_joint_trajectory_controller",
            default_value="false",
            description="Flag to enable JointTrajectoryController Gazebo plugin for manipulator.",
        ),
        DeclareLaunchArgument(
            "gazebo_joint_state_publisher",
            default_value="false",
            description="Flag to enable JointStatePublisher Gazebo plugin for all joints.",
        ),
        DeclareLaunchArgument(
            "gazebo_pose_publisher",
            default_value="true",
            description="Flag to enable PosePublisher Gazebo plugin for true pose of robot.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("lunalab_summit_xl_gen_description"),
                "rviz",
                "view.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
