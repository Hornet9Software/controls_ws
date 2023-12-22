import numpy as np
import rclpy
from controls_core.driver import AttitudeControl
from controls_core.params import UPTHRUST, rollPID, yawPID
from controls_core.thruster_allocator import ThrustAllocator
from imu_msg.msg import Imu
from rclpy.node import Node
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()
attitudeControl = AttitudeControl(rollPID, yawPID)

# [Roll, Pitch, Yaw]
targetAttRPY = np.array([0, 0, 0])


class AttitudeControlTestPublisher(Node):
    def __init__(self):
        super().__init__("attitude_control_test_publisher_node")
        self.state_subscriber = self.create_subscription(
            Imu, "/sensors/imu", self.attitudeControl, 10
        )

    def attitudeControl(self, msg: Imu):
        currAttRPY = [msg.roll_pitch_yaw.x, msg.roll_pitch_yaw.y, msg.roll_pitch_yaw.z]

        angular_acc = attitudeControl.getAttitudeCorrection(
            currAttRPY=currAttRPY, targetAttRPY=targetAttRPY
        )
        self.get_logger().info(f"PID angular acceleration: {angular_acc}")
        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs([0, 0, UPTHRUST], angular_acc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeControlTestPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()
