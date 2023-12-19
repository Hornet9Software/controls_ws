import time

import numpy as np
import pandas as pd
import rclpy
from controls_core.utilities import quat_to_list
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

FILEPATH = "test.csv"
duration = 10


class IMUSubscriber(Node):
    def __init__(self):
        super().__init__("IMU_zeroing")
        self.rpyAve = np.array([0, 0, 0])
        self.create_subscription(Imu, "/sensors/imu", self.imu_callback, 10)
        self.count = 0
        self.create_timer(duration, self.saveZeroing)

    def imu_callback(self, msg):
        rpy = np.array(euler_from_quaternion(quat_to_list(msg.orientation)))
        self.get_logger().info(f"RPY: {rpy}")
        self.rpyAve = (self.count * self.rpyAve + rpy) / (self.count + 1)
        self.count += 1

    def saveZeroing(self):
        self.get_logger().info(f"Shutting down node after {duration} seconds.")

        df = pd.DataFrame(self.rpyAve)
        df.to_csv(FILEPATH, index=False, header=False)
        self.get_logger().info(f"Data has been written to {FILEPATH}")


def main(args=None):
    rclpy.init(args=args)
    sub = IMUSubscriber()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()
