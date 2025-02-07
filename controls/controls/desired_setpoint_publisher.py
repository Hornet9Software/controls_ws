#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import CVObject
from geometry_msgs.msg import Pose, Quaternion, Point
from tf_transformations import quarternion_from_euler

import math


class SetpointPublisher(Node):
    def __init__(self):
        super().__init__("SetpointPublisher")
        self.listeners = {}
        self.listeners["gate_listener"] = self.create_subscription(
            CVObject, "/goal/gate", self.cv_callback, 10
        )

    def cv_callback(self, msg):
        d = msg.distance
        theta = msg.bearing

        x = d * math.cos(theta)
        y = d * math.sin(theta)
        z = 0.0

        r = 0.0
        p = 0.0
        y = theta

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = quarternion_from_euler(r, p, y)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
