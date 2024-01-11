#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


# This function is always needed
def generate_launch_description():
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch/foxglove_bridge_launch.xml",
            )
        )
    )

    # dummy_data = Node(package="simulation", executable="dummy_data")

    control_signals = Node(package="controls_core", executable="control_signals")

    simulation = Node(package="simulation", executable="simulation")

    movement_test = Node(package="simulation", executable="movement_test_sim")

    # robot_node = Node(
    #     namespace="core",
    #     package="params_pkg",
    #     executable="robot_node",
    #     parameters=[
    #         {
    #             "robot_name": "RobotA",
    #             "max_speed": 4.2,
    #             "waypoints": ["Home", "Room 1", "Corridor", "Home"],
    #         }
    #     ],
    # )

    # Launch Foxglove Studio to monitor data
    # foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    # Add the nodes and the process to the LaunchDescription list
    ld = [foxglove_bridge, control_signals, simulation, movement_test]
    # foxglove_studio]

    return LaunchDescription(ld)
