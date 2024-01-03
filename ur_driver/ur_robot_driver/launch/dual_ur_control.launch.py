import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    right_ur_type = LaunchConfiguration('right_ur_type')     
    right_robot_ip = LaunchConfiguration('right_robot_ip') 
    right_controller_file = LaunchConfiguration('right_controller_file') 
    right_tf_prefix = LaunchConfiguration('right_tf_prefix') 
    right_script_command_port = LaunchConfiguration('right_script_command_port')# default is 50004
    right_reverse_port = LaunchConfiguration('right_reverse_port')# default is 50001
    right_script_sender_port = LaunchConfiguration('right_script_sender_port')# default is 50002
    right_trajectory_port = LaunchConfiguration('right_trajectory_port')# default is 50003



    left_ur_type = LaunchConfiguration('left_ur_type')    
    left_robot_ip = LaunchConfiguration('left_robot_ip') 
    left_controller_file = LaunchConfiguration('left_controller_file') 
    left_tf_prefix = LaunchConfiguration('left_tf_prefix') 
    left_script_command_port = LaunchConfiguration('left_script_command_port')# default is 50004
    left_reverse_port = LaunchConfiguration('left_reverse_port')# default is 50001
    left_script_sender_port = LaunchConfiguration('left_script_sender_port')# default is 50002
    left_trajectory_port = LaunchConfiguration('left_trajectory_port')# default is 50003



    right_ur_type_arg = DeclareLaunchArgument(
            "right_ur_type",
            default_value='ur3',
    )
    right_robot_ip_arg = DeclareLaunchArgument(
            "right_robot_ip",
            default_value='192.168.8.195',
    )
    right_controller_file_arg = DeclareLaunchArgument(
            "right_controller_file",
            default_value="ur_controllers.yaml",
    )
    right_tf_prefix_arg = DeclareLaunchArgument(
            "right_tf_prefix",
            default_value="right_",
    )
    right_script_command_port_arg =  DeclareLaunchArgument(
            "right_script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    right_reverse_port_arg = DeclareLaunchArgument(
            "right_reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    right_script_sender_port_arg = DeclareLaunchArgument(
            "right_script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    right_trajectory_port_arg = DeclareLaunchArgument(
            "right_trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        )



    left_ur_type_arg = DeclareLaunchArgument(
            "left_ur_type",
            default_value='ur3',
    )
    left_robot_ip_arg = DeclareLaunchArgument(
            "left_robot_ip",
            default_value='192.168.8.196',
    )
    left_controller_file_arg = DeclareLaunchArgument(
            "left_controller_file",
            default_value="ur_controllers.yaml",
    )
    left_tf_prefix_arg = DeclareLaunchArgument(
            "left_tf_prefix",
            default_value="left_",
    )
    left_script_command_port_arg =  DeclareLaunchArgument(
            "left_script_command_port",
            default_value="60004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    left_reverse_port_arg = DeclareLaunchArgument(
            "left_reverse_port",
            default_value="60001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    left_script_sender_port_arg = DeclareLaunchArgument(
            "left_script_sender_port",
            default_value="60002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    left_trajectory_port_arg = DeclareLaunchArgument(
            "left_trajectory_port",
            default_value="60003",
            description="Port that will be opened for trajectory control.",
        )
  


    robot_description_content = ParameterValue(Command
    (
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "dual_ur.urdf.xacro"]),
            " ",
        ]
    ),
    value_type=str
    )
    robot_description = {"robot_description": robot_description_content}
    ur_robot_driver_path = get_package_share_directory('ur_robot_driver')



    right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_robot_driver_path, 'launch', 'ur_control.launch.py')),
        launch_arguments={'ur_type': right_ur_type,
                          'robot_ip': right_robot_ip,
                          'controllers_file': right_controller_file,
                          'tf_prefix': right_tf_prefix,
                          'script_command_port': right_script_command_port,
                          'trajectory_port': right_trajectory_port,
                          'reverse_port': right_reverse_port,
                          'script_sender_port': right_script_sender_port,
                          }.items())
    
    right_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('right'),
         right,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        )         
      ]
    )



    left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_robot_driver_path, 'launch', 'ur_control.launch.py')),
        launch_arguments={'ur_type': left_ur_type,
                          'robot_ip': left_robot_ip,
                          'controllers_file': left_controller_file,
                          'tf_prefix': left_tf_prefix,
                          'script_command_port': left_script_command_port,
                          'trajectory_port': left_trajectory_port,
                          'reverse_port': left_reverse_port,
                          'script_sender_port': left_script_sender_port,
                          }.items())
    
    left_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('left'),
         left,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description]
        )         
      ]
    )



    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "rviz", "view_robot_for_driver.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    

    return LaunchDescription([
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_right',
            output='screen',
            arguments=['/right/joint_states', '/joint_states'],
        ), 
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_left',
            output='screen',
            arguments=['/left/joint_states', '/joint_states'],
        ),       
        right_ur_type_arg,
        right_robot_ip_arg,
        right_controller_file_arg,
        right_tf_prefix_arg,
        right_script_command_port_arg,
        right_trajectory_port_arg,
        right_reverse_port_arg,
        right_script_sender_port_arg,
        left_ur_type_arg,
        left_robot_ip_arg,
        left_controller_file_arg,
        left_tf_prefix_arg,
        left_script_command_port_arg,
        left_trajectory_port_arg,
        left_reverse_port_arg,
        left_script_sender_port_arg,
        #rviz_node,# if you don't want to launch the rviz2 to show the robot state, comment it
        right_with_namespace,
        left_with_namespace
    ])
