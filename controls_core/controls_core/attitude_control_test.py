import numpy as np
import rclpy
from controls_core.driver import AttitudeControl
from controls_core.thruster_allocator import ThrustAllocator
from controls_core.utilities import quat_to_list
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()
attitudeControl = AttitudeControl()

# [Roll, Pitch, Yaw]
targetAttRPY = np.array([0, 0, 0])
ROLL_IDX = 0
YAW_IDX = 2


class AttitudeControlTestPublisher(Node):
    def __init__(self):
        super().__init__("attitude_control_test_publisher_node")
        self.state_subscriber = self.create_subscription(
            Odometry, "/state", self.attitudeControl, 10
        )

    def attitudeControl(self, msg: Odometry):
        currAttQuat = msg.pose.pose.orientation
        currAttRPY = euler_from_quaternion(quat_to_list(currAttQuat))

        angular_acc = attitudeControl.getAttitudeCorrection(
            currAttRPY=currAttRPY, targetAttRPY=targetAttRPY
        )
        self.get_logger().info(
            f"Deviation (Target - Curr): {targetAttRPY - currAttRPY}"
        )
        self.get_logger().info(f"PID angular acceleration: {angular_acc}")
        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs([0, 0, 0], angular_acc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args=None):
    rclpy.init(args=args)
    publisher = AttitudeControlTestPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
