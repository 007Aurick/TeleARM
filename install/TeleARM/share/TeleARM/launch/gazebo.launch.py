import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('TeleARM')
    urdf_path = os.path.join(pkg_share, 'urdf', 'telearm.urdf.xacro')

    # Strip XML comments. gazebo_ros2_control on Humble crashes when
    # robot_description has comments containing ": " (colons), so the
    # controller_manager never starts and the robot never moves.
    urdf_xml = subprocess.check_output(['xacro', urdf_path], text=True)
    urdf_xml = re.sub(r'<!--.*?-->', '', urdf_xml, flags=re.DOTALL)

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'world': '/home/auric/ros2_ws/TeleARM/worlds/storage_warehouse.world',
        }.items(),
    )

    # Plain gzclient: gazebo_ros's gzclient.launch.py hardcodes
    # libgazebo_ros_eol_gui.so, which segfaults on this WSL setup.
    gzclient_process = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': ParameterValue(urdf_xml, value_type=str),
                'use_sim_time': True,
            },
        ],
        output='screen',
    )

    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-z', '0.5',
            '-timeout', '180.0',
        ],
        output='screen',
    )

    # IMPORTANT: use -s (sim time) — Gazebo/controller_manager run on /clock.
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 3 && ros2 control load_controller -s --spin-time 30 '
            '--set-state active joint_state_broadcaster',
        ],
        output='screen',
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 1 && ros2 control load_controller -s --spin-time 30 '
            '--set-state active diff_drive_base_controller',
        ],
        output='screen',
    )

    load_gripper_controller = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'sleep 1 && ros2 control load_controller -s --spin-time 30 '
            '--set-state active gripper_controller',
        ],
        output='screen',
    )

    diff_drive_publisher_node = Node(
        package='TeleARM',
        executable='diff_drive_publisher.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    gripper_publisher_node = Node(
        package='TeleARM',
        executable='gripper_publisher.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # spawn → broadcaster → diff_drive → gripper → both publishers
    load_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_spawn_node,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    load_diff_drive_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_diff_drive_base_controller],
        )
    )

    load_gripper_after_diff = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_diff_drive_base_controller,
            on_exit=[load_gripper_controller],
        )
    )

    start_publishers_after_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_gripper_controller,
            on_exit=[diff_drive_publisher_node, gripper_publisher_node],
        )
    )

    return [
        gzserver_launch,
        gzclient_process,
        robot_state_publisher_node,
        robot_spawn_node,
        load_broadcaster_after_spawn,
        load_diff_drive_after_broadcaster,
        load_gripper_after_diff,
        start_publishers_after_gripper,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
