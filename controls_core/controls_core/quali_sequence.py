import numpy as np
import rclpy
import time
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
attitudeControl = AttitudeControl(rollPID, pitchPID, yawPID)
positionControl = PositionControl(distancePID, lateralPID, depthPID)

targetXYZ = np.array([0, 0, -1.2])
targetRPY = np.array([0, 0, 0])

class DepthControlTest(Node):
    def __init__(self, targetXYZ=targetXYZ, targetRPY=targetRPY, testRPYControl=False):
        super().__init__("depth_control_test")

        self.targetXYZ = targetXYZ
        self.targetRPY = targetRPY

        self.testRPYControl = testRPYControl

        self.create_timer(0.1, self._controlLoop)
        self.first = True

        self.currXYZ = [0.0, 0.0, 0.0]
        self.currRPY = [0.0, 0.0, 0.0]

        self.depthListener = self.create_subscription(
            Float32, "/sensors/depth", self._depthProcessing, 10
        )

        self.imuListener = self.create_subscription(
            Imu, "/sensors/imu", self._imuProcessing, 10
        )

        self.init_time = time.time()

    def _imuProcessing(self, msg: Imu):
        self.currRPY = [
            msg.roll_pitch_yaw.x,
            msg.roll_pitch_yaw.y,
            msg.roll_pitch_yaw.z,
        ]

    def _depthProcessing(self, msg: Float32):
        self.currXYZ[2] = msg.data

    def _controlLoop(self):
        if self.first:
            self.first = False
            self.init_time = time.time()

        if self.testRPYControl:
            self.currRPY = attitudeControl.correctIMU(self.currRPY)
            angularAcc = attitudeControl.getAttitudeCorrection(
                currRPY=self.currRPY, targetRPY=self.targetRPY
            )
            self.get_logger().info(f"PID angular acceleration: {angularAcc}")
        else:
            angularAcc = [0.0, 0.0, 0.0]

        linearAcc = positionControl.getPositionCorrection(
            currXYZ=self.currXYZ, targetXYZ=targetXYZ
        )

        curr_time = time.time()
        delta_t = curr_time - self.init_time

        if delta_t < 5:
            linearAcc[1] = 0.0
        elif delta_t < 5.5:
            linearAcc[1] = 1.0
        elif delta_t >= 5.5 and delta_t < 6.0:
            linearAcc[1] = -1.0
        else:
            linearAcc[1] = 0.0

        self.get_logger().info(f"Curr Depth: {self.currXYZ[2]}")
        self.get_logger().info(f"Target Depth: {targetXYZ[2]}")
        self.get_logger().info(f"Correction: {linearAcc}")

        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs(linearAcc, angularAcc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args=None):
    rclpy.init(args=args)
    depthTest = DepthControlTest(testRPYControl=True)

    try:
        rclpy.spin(depthTest)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()
