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

    madgwick_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="madgwick_filter",
        output="screen",
        parameters=[{
            "use_sim_time":True,
            "use_mag":False,
            "world_frame":"enu",
            "publish_tf":True,
            "gain":0.05,
            "zeta":0.01,
        }]
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
    ld.add_action(madgwick_filter)
    ld.add_action(balance_bot_controller)
    
    
    return ld