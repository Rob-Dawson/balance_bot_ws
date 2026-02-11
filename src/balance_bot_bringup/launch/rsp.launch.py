#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    balance_bot_directory = get_package_share_directory("balance_bot_description")
    balance_bot_xacro = os.path.join(balance_bot_directory, "urdf", "balance_bot.urdf.xacro")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=balance_bot_xacro,
        description="Path to the robot xacro file"
    )

    model = LaunchConfiguration("model")
    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
        )

    robot_state_pub_params = {"robot_description": robot_description, "use_sim_time":use_sim_time}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_state_pub_params],
    )

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use Gazebo clock"
    )


    ld = LaunchDescription()
    ld.add_action(model_arg)
    ld.add_action(declare_use_sim_time)
    ld.add_action(robot_state_publisher)

    return ld