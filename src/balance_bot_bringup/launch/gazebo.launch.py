#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    balance_bot_bringup_dir = get_package_share_directory("balance_bot_bringup")
    balance_bot_description_dir = get_package_share_directory("balance_bot_description")
    balance_bot_controller_dir = get_package_share_directory("balance_bot_controller")
    gazebo_dir = get_package_share_directory("ros_gz_sim")

    rsp_launch = os.path.join(
        balance_bot_bringup_dir,
        "launch", 
        "rsp.launch.py"
    )

    balance_bot_controller_launch = os.path.join(
        balance_bot_controller_dir,
        "launch", 
        "balance_bot_controller.launch.py"
    )

    default_world = os.path.join(
        balance_bot_description_dir,
        "worlds", 
        "empty.sdf"
    )

    gazebo_launch = os.path.join(
        gazebo_dir,
        "launch",
        "gz_sim.launch.py"
    )

    bridge_params = os.path.join(
        balance_bot_bringup_dir,
        "config", 
        "gazebo_bridge.yaml"
    )


    ## Launch rsp
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            rsp_launch
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    balance_bot_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            balance_bot_controller_launch
        )
    )

    ## Gazebo Stuff
    declare_world = DeclareLaunchArgument(
        "world", 
        default_value=default_world,
        description="World to load"
    )
    world = LaunchConfiguration("world")


    gazebo_params = {"gz_args": ["-v 4 ", world],"on_exit_shutdown":"true",}
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch
            ),
            launch_arguments=
                gazebo_params.items(),
        )

    spawn_arguments = ["-name", "balance_bot",
                       "-topic","robot_description", 
                       "-z", "0.0300",
                       ]
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=spawn_arguments,
        output="screen"
    )

    gazebo_bridge_params = [
        {"config_file":bridge_params}
        ]
    gazebo_bridge= Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=gazebo_bridge_params
    )


    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(rsp)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(balance_bot_controller)
    ld.add_action(gazebo_bridge)
    return ld
