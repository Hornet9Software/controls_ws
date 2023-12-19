import rclpy
from controls_core.thruster_allocator import ThrustAllocator
from rclpy.node import Node
from thrusters.thrusters import ThrusterControl


class AttitudeControlTestPublisher(Node):
    def __init__(self):
        super().__init__("movement_test_publisher_node")
        self.timer = self.create_timer(1.0, self.movementTest)

    def movementTest(self):
        thrusterControl = ThrusterControl()
        solver = ThrustAllocator()

        # FL-FR-ML-MR-RL-RR
        thrustValues = solver.getThrustPWMs(self.lin_acc, self.angular_acc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(movementPublisher):
    rclpy.init(args=None)
    rclpy.spin(movementPublisher)
    movementPublisher.destroy_node()
    rclpy.shutdown()
