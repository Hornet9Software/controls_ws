import numpy as np
import rclpy
from controls_core.thruster_allocator import ThrustAllocator
from rclpy.node import Node
from thrusters.thrusters import ThrusterControl

LIN_ACC_MAG = 1.0

MOVE_LEFT = [-1, 0, 0]
MOVE_RIGHT = [1, 0, 0]
MOVE_FRONT = [0, 1, 0]
MOVE_BACK = [0, -1, 0]
MOVE_UP = [0, 0, 1]
MOVE_DOWN = [0, 0, -1]


thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()


class MovementTestPublisher(Node):
    def __init__(self, lin_acc, angular_acc):
        super().__init__("movement_test_publisher_node")
        self.timer = self.create_timer(1.0, self.movementTest)
        self.lin_acc = lin_acc
        self.angular_acc = angular_acc

    def movementTest(self):
        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs(self.lin_acc, self.angular_acc)
        self.get_logger().info(f"{thrustValues}")
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args, direction, magnitude):
    rclpy.init(args=args)
    movementPublisher = MovementTestPublisher(
        lin_acc=magnitude * np.array(direction), angular_acc=[0, 0, 0]
    )
    rclpy.spin(movementPublisher)
    movementPublisher.destroy_node()
    rclpy.shutdown()


def moveLeft(args=None):
    main(args, MOVE_LEFT, LIN_ACC_MAG)


def moveRight(args=None):
    main(args, MOVE_RIGHT, LIN_ACC_MAG)


def moveFront(args=None):
    main(args, MOVE_FRONT, LIN_ACC_MAG)


def moveBack(args=None):
    main(args, MOVE_BACK, LIN_ACC_MAG)


def moveUp(args=None):
    main(args, MOVE_UP, LIN_ACC_MAG)


def moveDown(args=None):
    main(args, MOVE_DOWN, LIN_ACC_MAG)
