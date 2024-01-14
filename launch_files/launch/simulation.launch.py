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

    dummy_data = Node(package="simulation", executable="dummy_data")

    # cv_signals = Node(package="control_signals", executable="cv_signals")

    imu_signals = Node(package="control_signals", executable="imu_signals")

    pid_manager = Node(package="simulation", executable="pid_manager_sim")

    # simulation = Node(package="simulation", executable="simulation")

    movement_test = Node(package="simulation", executable="movement_test_sim")

    # Launch Foxglove Studio to monitor data
    # foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    # Add the nodes and the process to the LaunchDescription list
    # ld = [camera_launch, cv_signals, imu_signals, pid_manager, movement_test]
    ld = [foxglove_bridge, dummy_data, imu_signals, pid_manager, movement_test]
    # foxglove_studio]

    return LaunchDescription(ld)
