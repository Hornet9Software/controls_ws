#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    include_launch_description,
)
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource


# This function is always needed
def generate_launch_description():
    # foxglove_bridge = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("foxglove_bridge"),
    #             "launch/foxglove_bridge_launch.xml",
    #         )
    #     )
    # )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("camera"), "launch/pooltest.launch.py"
            )
        )
    )

    cv_signals = Node(package="control_signals", executable="cv_signals")

    imu_signals = Node(package="control_signals", executable="imu_signals")

    pid_manager = Node(package="controls_core", executable="pid_manager")

    movement_test = Node(package="tasks", executable="movement_test")

    # Launch Foxglove Studio to monitor data
    # foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    # Add the nodes and the process to the LaunchDescription list
    ld = [camera_launch, cv_signals, imu_signals, pid_manager, movement_test]

    return LaunchDescription(ld)
