#!/usr/bin/env python3
import time

import numpy as np
import pandas as pd
import rclpy
from imu_msg.msg import Imu
from rclpy.node import Node

FILEPATH = "test.csv"
duration = 10


class IMUSubscriber(Node):
    def __init__(self):
        super().__init__("IMU_zeroing")
        self.rpyAve = np.array([0, 0, 0])
        self.create_subscription(Imu, "/sensors/imu", self.imu_callback, 10)
        self.count = 0
        self.end_t = time.time() + 10

    def imu_callback(self, msg):
        rpy = np.array(
            [msg.roll_pitch_yaw.x, msg.roll_pitch_yaw.y, msg.roll_pitch_yaw.z]
        )
        self.get_logger().info(f"RPY: {rpy}")
        self.rpyAve = (self.count * self.rpyAve + rpy) / (self.count + 1)
        self.count += 1
        if time.time() > self.end_t:
            print("done")
            self.saveZeroing()
            self.destroy_node()
            rclpy.shutdown()

    def saveZeroing(self):
        self.get_logger().info(f"Shutting down node after {duration} seconds.")

        df = pd.DataFrame(self.rpyAve)
        df.to_csv(FILEPATH, index=False, header=False)
        self.get_logger().info(f"Data has been written to {FILEPATH}")


def main(args=None):
    rclpy.init(args=args)
    sub = IMUSubscriber()
    rclpy.spin(sub)
    # sub.destroy_node()
    # rclpy.shutdown()
