#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager",
                   "/controller_manager"],
    )

    balance_bot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["balance_bot_controller",
                   "--controller-manager", 
                   "/controller_manager"]
    )
    
    ld = LaunchDescription()
    ld.add_action(joint_state_broadcaster)
    ld.add_action(balance_bot_controller)
    
    
    return ld