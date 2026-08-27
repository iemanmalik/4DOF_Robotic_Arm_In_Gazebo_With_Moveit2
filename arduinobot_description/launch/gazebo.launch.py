import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_description = get_package_share_directory(
        'arduinobot_description'
    )

    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim'
    )

    model_file = os.path.join(
        pkg_description,
        'urdf',
        'arduinobot.urdf.xacro'
    )

    # Robot description generated from Xacro
    robot_description = ParameterValue(
        Command([
            'xacro ',
            model_file,
            ' is_sim:=true'
        ]),
        value_type=str
    )

    # Publish robot state / TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    # Start Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf'
        }.items()
    )

    # Spawn robot from /robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'arduinobot',
            '-topic',
            'robot_description',
            '-x',
            '0',
            '-y',
            '0',
            '-z',
            '0'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
    ])
