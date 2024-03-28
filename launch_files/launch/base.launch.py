import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

"""
This file launches all background/base nodes needed.
"""


def generate_launch_description():
    # foxglove_bridge = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("foxglove_bridge"),
    #             "launch/foxglove_bridge_launch.xml",
    #         )
    #     )
    # )

    # camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("camera"), "launch/pooltest.launch.py"
    #         )
    #     )
    # )

    cv_signals = Node(package="control_signals", executable="cv_signals")

    # imu_signals = Node(package="control_signals", executable="imu_signals")

    # Launch Foxglove Studio to monitor data
    # foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    # Add the nodes and the process to the LaunchDescription list
    ld = [
        cv_signals,
        # imu_signals,
    ]
    # foxglove_studio]

    return LaunchDescription(ld)
