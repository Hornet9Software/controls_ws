import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class SensorFusion(Node):
    """
    This node listens to all sensors inputs and uses a Kalman filter
    to perform sensor fusion for localisation.

    Publishes to "/state".
    """

    def __init__(self):
        super().__init__("sensor_fusion")


def main(args=None):
    rclpy.init(args=args)
    sensorFusion = SensorFusion()

    try:
        rclpy.spin(sensorFusion)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
