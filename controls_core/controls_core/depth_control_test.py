import numpy as np
import rclpy
from controls_core.attitude_control import AttitudeControl
from controls_core.params import *
from controls_core.position_control import PositionControl
from controls_core.thruster_allocator import ThrustAllocator
from imu_msg.msg import Imu
from rclpy.node import Node
from std_msgs.msg import Float32
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()
attitudeControl = AttitudeControl(rollPID, yawPID)
positionControl = PositionControl(distancePID, lateralPID, depthPID)

targetXYZ = np.array([0, 0, -1])
targetRPY = np.array([0, 0, 0])


class DepthControlTest(Node):
    def __init__(self, targetXYZ=targetXYZ, targetRPY=targetRPY, testRPYControl=True):
        super().__init__("depth_control_test")

        self.targetXYZ = targetXYZ
        self.targetRPY = targetRPY

        self.testRPYControl = testRPYControl

        self.create_timer(0.1, self._controlLoop)

        self.currXYZ = [0.0, 0.0, 0.0]
        self.currRPY = [0.0, 0.0, 0.0]

        self.depthListener = self.create_subscription(
            Float32, "/sensors/depth", self._depthProcessing, 10
        )

        self.imuListener = self.create_subscription(
            Imu, "/sensors/imu", self._imuProcessing, 10
        )

    def _imuProcessing(self, msg: Imu):
        self.currRPY = [
            msg.roll_pitch_yaw.x,
            msg.roll_pitch_yaw.y,
            msg.roll_pitch_yaw.z,
        ]

    def _depthProcessing(self, msg: Float32):
        self.currXYZ[2] = msg.data

    def _controlLoop(self):
        if self.testRPYControl:
            angularAcc = attitudeControl.getAttitudeCorrection(
                currRPY=self.currRPY, targetRPY=self.targetRPY
            )
            self.get_logger().info(f"PID angular acceleration: {angularAcc}")
        else:
            angularAcc = [0.0, 0.0, 0.0]

        linearAcc = positionControl.getPositionCorrection(
            currXYZ=self.currXYZ, targetXYZ=targetXYZ
        )

        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs(linearAcc, angularAcc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args=None):
    rclpy.init(args=args)
    depthTest = DepthControlTest()

    try:
        rclpy.spin(depthTest)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()
