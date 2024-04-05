#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    include_launch_description,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


# This function is always needed
def generate_launch_description():
    qualification_run = Node(package="tasks", executable="qualification_run")

    ld = [qualification_run]

    return LaunchDescription(ld)
