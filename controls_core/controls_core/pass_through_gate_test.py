import numpy as np
import rclpy
from controls_core.chase_object_node import ChaseObjectNode, ChaseObjectParams
from controls_core.params import *
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()

targetXYZ = np.array([0, 0, -1.0])
targetRPY = [0, 0, 0]

gate_params = ChaseObjectParams(object_name="gate", threshold_dist=2.0, linear_acc=1.5)


def main(args=None):
    rclpy.init(args=args)
    test_node = ChaseObjectNode(targetXYZ, targetRPY, gate_params, testRPYControl=True)

    try:
        rclpy.spin(test_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()
