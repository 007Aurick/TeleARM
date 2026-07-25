"""Spawn TeleARM into an ALREADY RUNNING Gazebo (e.g. AWS warehouse).

Do NOT use gazebo.launch.py for this — that starts a second Gazebo.
"""
import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('TeleARM')
    urdf_path = os.path.join(pkg_share, 'urdf', 'telearm.urdf.xacro')

    urdf_xml = subprocess.check_output(['xacro', urdf_path], text=True)
    urdf_xml = re.sub(r'<!--.*?-->', '', urdf_xml, flags=re.DOTALL)

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

    # Spawn into the warehouse (open area near origin; tweak if you clip a shelf)
    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'telearm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-timeout', '180.0',
        ],
        output='screen',
    )

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

    diff_drive_publisher_node = Node(
        package='TeleARM',
        executable='diff_drive_publisher.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return [
        robot_state_publisher_node,
        robot_spawn_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_spawn_node,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_base_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_diff_drive_base_controller,
                on_exit=[diff_drive_publisher_node],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
