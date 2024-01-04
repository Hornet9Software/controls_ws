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

    control_signals = Node(package="controls_core", executable="control_signals")

    # simulation = Node(package="simulation", executable="simulation")

    movement_test = Node(package="tasks", executable="movement_test")

    # Launch Foxglove Studio to monitor data
    # foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    # Add the nodes and the process to the LaunchDescription list
    ld = [control_signals, movement_test]

    return LaunchDescription(ld)
