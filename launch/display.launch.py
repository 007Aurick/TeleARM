import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_path = get_package_share_directory('TeleARM')
    default_urdf = os.path.join(pkg_path, 'urdf', 'telearm.urdf.xacro')

    configs = context.launch_configurations
    urdf_path = configs.get('urdf', default_urdf)
    if configs.get('model', default_urdf) != default_urdf:
        urdf_path = configs['model']
    elif configs.get('urdf', default_urdf) != default_urdf:
        urdf_path = configs['urdf']

    urdf_path = os.path.expanduser(urdf_path)
    if not os.path.isabs(urdf_path):
        urdf_path = os.path.join(pkg_path, urdf_path)

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str,
    )

    rviz_config = os.path.join(pkg_path, 'rviz', 'telearm.rviz')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ]


def generate_launch_description():
    pkg_path = get_package_share_directory('TeleARM')
    default_urdf = os.path.join(pkg_path, 'urdf', 'telearm.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf',
            default_value=default_urdf,
            description='Path to the URDF/xacro file',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_urdf,
            description='Alias for urdf (e.g. model:=urdf/telearm.urdf.xacro)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
