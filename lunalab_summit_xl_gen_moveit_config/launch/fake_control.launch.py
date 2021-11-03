#!/usr/bin/env -S ros2 launch
"""Example of planning with MoveIt2 and executing motions using fake ROS 2 controllers within RViz2"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from os import path
from typing import List, Dict
import yaml


def generate_launch_description():

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    moveit_config_package = "lunalab_summit_xl_gen_moveit_config"
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_position_margin = LaunchConfiguration("safety_position_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    high_quality_mesh = LaunchConfiguration("high_quality_mesh")
    ros2_control = LaunchConfiguration("ros2_control")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # URDF
    _robot_description_xml = Command(
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
            " ",
            "ros2_control:=",
            ros2_control,
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # SRDF
    _robot_description_semantic_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package),
                                  "srdf", "lunalab_summit_xl_gen.srdf.xacro"]),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description_semantic = {"robot_description_semantic":
                                  _robot_description_semantic_xml}

    # Kinematics
    kinematics = load_yaml(moveit_config_package,
                           path.join("config", "kinematics.yaml"))

    # Joint limits
    joint_limits = {"robot_description_planning": load_yaml(moveit_config_package,
                                                            path.join("config", "joint_limits.yaml"))}

    # Planning pipeline
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            # TODO: Re-enable `default_planner_request_adapters/AddRuckigTrajectorySmoothing` once its issues are resolved
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }
    _ompl_yaml = load_yaml(moveit_config_package,
                           path.join("config", "ompl_planning.yaml"))
    planning_pipeline["ompl"].update(_ompl_yaml)

    # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # MoveIt controller manager
    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": load_yaml(moveit_config_package,
                                                      path.join("config", "controllers.yaml")),
    }

    # Trajectory execution
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # ROS 2 controller manager
    ros2_controllers_path = path.join(get_package_share_directory(moveit_config_package),
                                      "config", "ros2_controllers.yaml")
    ros2_controllers_yaml = parse_yaml(ros2_controllers_path)

    # List of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[robot_description,
                        {"use_sim_time": use_sim_time}],
        ),
        # ros2_control_node
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[robot_description,
                        ros2_controllers_path,
                        {"use_sim_time": use_sim_time}],
        ),
        # move_group
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="moveit_ros_move_group",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics,
                joint_limits,
                planning_pipeline,
                trajectory_execution,
                planning_scene_monitor_parameters,
                moveit_controller_manager,
                {"use_sim_time": use_sim_time}],
        ),
        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["--display-config", rviz_config,
                       "--ros-args", "--log-level", log_level],
            parameters=[robot_description,
                        robot_description_semantic,
                        kinematics,
                        planning_pipeline,
                        joint_limits,
                        {"use_sim_time": use_sim_time}],
        ),
    ]

    # Add nodes for loading controllers
    for controller in parse_controllers(ros2_controllers_yaml):
        nodes.append(
            # controller_manager_spawner
            Node(
                package="controller_manager",
                executable="spawner",
                name="controller_manager_spawner_" + controller,
                output="log",
                arguments=[controller,
                           "--ros-args", "--log-level", log_level],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        )

    return LaunchDescription(declared_arguments + nodes)


def load_yaml(package_name: str, file_path: str):
    """
    Load yaml configuration based on package name and file path relative to its share.
    """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)


def parse_yaml(absolute_file_path: str):
    """
    Parse yaml from file, given its absolute file path.
    """

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def parse_controllers(controller_manager: Dict) -> List[str]:
    """
    Parse a list of controllers from ros2 controller config.
    The following structure is assumed
        controller_manager: {
            ros__parameters: {
                update_rate: ()
                controller_0: {}
                controller_1: {}
                controller_n: {}
            }
        }
        ...
    """

    ros_params = controller_manager["controller_manager"]["ros__parameters"]
    controllers = list(ros_params.keys())
    controllers.remove("update_rate")
    return controllers


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Locations of robot resources
        DeclareLaunchArgument(
            "description_package",
            default_value="lunalab_summit_xl_gen_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("urdf",
                                    "lunalab_summit_xl_gen.urdf.xacro"),
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

        # Geometry
        DeclareLaunchArgument(
            "high_quality_mesh",
            default_value="true",
            description="Flag to select the high or low quality model.",
        ),

        # ROS 2 control
        DeclareLaunchArgument(
            "ros2_control",
            default_value="true",
            description="Flag to enable ros2 controllers for manipulator.",
        ),

        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(get_package_share_directory("lunalab_summit_xl_gen_moveit_config"),
                                    "rviz", "moveit.rviz"),
            description="Path to configuration for RViz2."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock."
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script."
        ),
    ]
