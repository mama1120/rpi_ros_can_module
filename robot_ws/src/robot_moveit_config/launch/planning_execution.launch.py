#    Copyright 2022 Christoph Hellmann Santos
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    share_dir = get_package_share_directory('robot_moveit_config')
    
    # ros2 nodes
    xacro_file = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    robot_control_config = PathJoinSubstitution(
        [FindPackageShare("robot_moveit_config"), "config", "ros2_controllers.yaml"]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_urdf}, robot_control_config],
        output="both",
    )
    
    # contoller spawner
    #robot_state_publisher_node = Node(
    #    package='robot_state_publisher',
    #    executable='robot_state_publisher',
    #    output="both",
    #    name='robot_state_publisher',
    #    parameters=[
    #        {'robot_description': robot_urdf}
    #    ]
    #)        
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    robot_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
    )
    
    #delayed_spawners = TimerAction(
    #	period=5.0,
    #	actions=[robot_state_publisher_node, joint_state_broadcaster_spawner, robot_arm_controller_spawner, hand_controller_spawner] #robot_state_publisher_node,
    #)
    
    # MoveIt
    moveit_config = MoveItConfigsBuilder("robot", package_name="robot_moveit_config").to_moveit_configs()
    
    virtual_joints = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
            )
        ),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/move_group.launch.py")
        ),
    )

    
    # custom UI Launch
    
    # Define the nodes to launch from robot_control package
    #move_home_node = Node(
    #    package='robot_control',
    #    executable='move_home',  # Assuming executable is named 'move_home'
    #    name='move_home_node',
    #    output='screen'
    #)

    #move_X_Z_node = Node(
    #    package='robot_control',
    #    executable='move_X_Z',  # Assuming executable is named 'move_X_Z'
    #    name='move_X_Z_node',
    #    output='screen'
    #)
    #joint_listener_node = Node(
    #package='robot_control',
    #executable='joint_listener',
    #name='joint_listener_node',
    #output='screen'
    #)
    
    pdo_sender_node = Node(
        package='joint_pdo_sender',
        executable='joint_state_to_pdo_save_state',
        name='pdo_sender_node',
        output='screen'
    )
    
    node_list = [
        control_node,
        joint_state_broadcaster_spawner,
        robot_arm_controller_spawner,
        hand_controller_spawner,
        virtual_joints,
        move_group,
        pdo_sender_node,
        #move_home_node,
        #move_X_Z_node,
        #joint_listener_node
    ]

    return LaunchDescription(node_list)
