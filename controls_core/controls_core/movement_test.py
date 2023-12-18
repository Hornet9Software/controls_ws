import rclpy
from controls_core.thruster_allocator import ThrustAllocator
from rclpy.node import Node
from thrusters.thrusters import ThrusterControl


class MovementTestPublisher(Node):
    def __init__(self, lin_acc, angular_acc):
        super().__init__("movement_test_publisher_node")
        self.timer = self.create_timer(1.0, self.movementTest)
        self.lin_acc = lin_acc
        self.angular_acc = angular_acc

    def movementTest(self):
        thrusterControl = ThrusterControl()
        solver = ThrustAllocator()

        # FL-FR-ML-MR-RL-RR
        thrustValues = solver.getThrustPWMs(self.lin_acc, self.angular_acc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


linAccMag = 1.0


def main(movementPublisher):
    rclpy.init(args=None)
    rclpy.spin(movementPublisher)
    movementPublisher.destroy_node()
    rclpy.shutdown()


def moveLeft():
    movementPublisher = MovementTestPublisher(
        [-linAccMag, 0.0, 0.0], angular_acc=[0.0, 0.0, 0.0]
    )
    main(movementPublisher)


def moveRight():
    movementPublisher = MovementTestPublisher(
        [-linAccMag, 0.0, 0.0], angular_acc=[0.0, 0.0, 0.0]
    )
    main(movementPublisher)


def moveFront():
    movementPublisher = MovementTestPublisher(
        [0.0, linAccMag, 0.0], angular_acc=[0.0, 0.0, 0.0]
    )
    main(movementPublisher)


def moveBack():
    movementPublisher = MovementTestPublisher(
        [0.0, -linAccMag, 0.0], angular_acc=[0.0, 0.0, 0.0]
    )
    main(movementPublisher)


def moveDown():
    movementPublisher = MovementTestPublisher(
        [0.0, 0, 0, -linAccMag], angular_acc=[0.0, 0.0, 0.0]
    )
    main(movementPublisher)
