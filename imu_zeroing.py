import time

import numpy as np
import pandas as pd
import rclpy
from controls_core.utilities import quat_to_list
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

count = 0
FILEPATH = ""
duration = 10


class IMUSubscriber(Node):
    def __init__(self):
        super.__init__("IMU_zeroing")
        self.rpyAve = np.array([0, 0, 0])
        self.create_subscription(Imu, "/sensors/imu", self.imu_callback, 10)
        self.shutdown_node_after_delay(duration)

    def imu_callback(self, msg):
        rpy = np.array(euler_from_quaternion(quat_to_list(msg.orientation)))
        self.rpyAve = (count * self.rpyAve + rpy) / (count + 1)
        count += 1

    def shutdown_node_after_delay(self, delay_seconds):
        time.sleep(delay_seconds)
        self.get_logger().info(
            "Shutting down node after {} seconds.".format(delay_seconds)
        )

        df = pd.DataFrame(self.rpyAve)
        df.to_csv(FILEPATH, index=False, header=False)
        self.get_logger().info(f"Data has been written to {FILEPATH}")

        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    sub = IMUSubscriber()
    rclpy.spin(sub)
