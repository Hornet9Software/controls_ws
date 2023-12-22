import numpy as np
import rclpy
from controls_core.driver import Driver
from controls_core.params import UPTHRUST
from controls_core.utilities import BACK, DOWN, FRONT, LEFT, RIGHT, UP

LIN_ACC_MAG = 1.0


def main(args, direction, magnitude):
    rclpy.init(args=args)
    node = Driver()
    linear_acc = magnitude * np.array(direction) + np.array([0, 0, UPTHRUST])
    node.drive(linear_acc, [0, 0, 0])

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()


def moveLeft(args=None):
    main(args, LEFT, LIN_ACC_MAG)


def moveRight(args=None):
    main(args, RIGHT, LIN_ACC_MAG)


def moveFront(args=None):
    main(args, FRONT, LIN_ACC_MAG)


def moveBack(args=None):
    main(args, BACK, LIN_ACC_MAG)
