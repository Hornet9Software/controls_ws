#!/usr/bin/env python3

import rclpy
import numpy as np
import math
import random

from std_msgs.msg import Float32, Float32MultiArray
from rclpy.node import Node


class DummyData(Node):
    def __init__(self, yoloFps=40, depthHz=50):
        super().__init__(node_name="dummy_data_publisher")
        self.yoloUpdatePeriod = 1 / yoloFps

        self.depthUpdatePeriod = 1 / depthHz

        self.yoloPublisher = self.create_publisher(
            Float32MultiArray, "/object/gate/yolo", 10
        )
        self.yoloTimer = self.create_timer(self.yoloUpdatePeriod, self.yoloDummyData)

        self.depthPublisher = self.create_publisher(Float32, "/sensors/depth", 10)
        self.depthTimer = self.create_timer(self.depthUpdatePeriod, self.depthDummyData)

    def yoloDummyData(self):
        xMin = random.uniform(0.2, 0.8)
        xMax = random.uniform(xMin, 1.0)
        x = (xMin + xMax) / 2.0
        w = xMax - xMin

        # Assuming AUV is centred vertically to gate
        y = 0.5
        h = random.uniform(0.2, 0.95)

        msg = Float32MultiArray()
        msg.data = [x, y, w, h]
        self.yoloPublisher.publish(msg)
        print("PUBLISHED BBOX:", msg.data)

    def depthDummyData(self):
        depth = random.uniform(-2.0, 0.0)
        msg = Float32()
        msg.data = depth
        self.depthPublisher.publish(msg)
        print("PUBLISHED DEPTH:", depth)


def main(args=None):
    rclpy.init(args=args)
    pub = DummyData()

    try:
        rclpy.spin(pub)
    except KeyboardInterrupt:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
