import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def get_robot_description():
    
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3", "default_kinematics.yaml"])
 
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "dual_ur.urdf.xacro"]),
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}    
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("dual_ur_moveit_config"), "config", "dual_arm.srdf"]
            ),
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    return robot_description_semantic
    
def get_robot_description_kinematics():
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"])
    return robot_description_kinematics
    
def get_robot_kinematics_params():
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3", "default_kinematics.yaml"])
    return kinematics_params


def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    robot_description_kinematics = get_robot_description_kinematics()
    demo_node = Node(
        package="dual_ur_moveit_api",
        executable="dual_ur_moveit_api",
        name="dual_ur_moveit_api_right",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )
    return launch.LaunchDescription([demo_node])
