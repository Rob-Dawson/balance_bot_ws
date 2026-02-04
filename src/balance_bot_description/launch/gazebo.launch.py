#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    balance_bot_directory = get_package_share_directory("balance_bot_description")
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            balance_bot_directory, "urdf", "balance_bot.urdf.xacro"
        ),
        description="Absolute path to the robot URDF",
    )
    print(model_arg)
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    default_world = os.path.join(balance_bot_directory, "worlds", "empty.sdf")
    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="World to load"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": ["r -v4 ", world, ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    delcare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use gazebo clock"
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "balance_bot",
            "-topic", "robot_description",
            "-z", "0.0300",
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(model_arg)
    ld.add_action(world_arg)
    ld.add_action(delcare_use_sim_time_cmd)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher)

    return ld