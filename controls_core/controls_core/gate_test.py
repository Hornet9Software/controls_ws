import time

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
attitudeControl = AttitudeControl(rollPID, pitchPID, yawPID)
positionControl = PositionControl(distancePID, lateralPID, depthPID)

targetXYZ = np.array([0, 0, -1])
targetRPY = np.array([0, 0, 0])


class MoveToGateTest(Node):
    def __init__(self, setDepth=-1.0):
        super().__init__("move_to_gate_test")
        self.create_timer(0.1, self._controlLoop)

        self.depthListener = self.create_subscription(
            Float32, "/sensors/depth", self._depthProcessing, 10
        )
        self.imuListener = self.create_subscription(
            Imu, "/sensors/imu", self._imuProcessing, 10
        )

        self.attitudeControl = AttitudeControl(
            rollPID=rollPID, pitchPID=pitchPID, yawPID=yawPID
        )
        self.currXYZ = [0.0, 0.0, 0.0]
        self.currRPY = [0.0, 0.0, 0.0]
        self.targetRPY = [0.0, 0.0, 0.0]
        self.targetXYZ = [0.0, 0.0, setDepth]

    def _imuProcessing(self, msg: Imu):
        self.currRPY = [
            msg.roll_pitch_yaw.x,
            msg.roll_pitch_yaw.y,
            msg.roll_pitch_yaw.z,
        ]

    def _depthProcessing(self, msg: Float32):
        self.currXYZ[2] = msg.data

    def _controlLoop(self, ud):
        # BEARING | target is bearing; current is 0
        # LATERAL | setpoint is 0; current is lateral
        # DISTANCE | setpoint is distance, current is 0
        while True:
            rclpy.spin_once(self.task_state)

            # self.logger.info("hello1")
            while (
                (self.state is None)
                or (self.depth is None)
                or (self.cv_data["gate"] is None)
            ):
                rclpy.spin_once(self.task_state)

            self.targetRPY = [0.0, 0.0, 0.0]
            self.targetXYZ = [0.0, 0.0, self.setDepth]
            self.currRPY = [
                self.state.angular_position.y,
                self.state.angular_position.x,
                0.0,
            ]
            self.currRPY = list(self.attitudeControl.correctIMU(self.currRPY))
            self.currRPY[2] = 0.0

            self.currXYZ = [0.0, 0.0, self.depth]

            if self.bearingControl:
                self.targetRPY[2] = self.cv_data["gate"]["bearing"]
                self.currRPY[2] = 0.0

            if self.lateralControl:
                self.targetXYZ[0] = 0.0
                self.currXYZ[0] = (
                    self.lateralDirection * self.cv_data["gate"]["lateral"]
                )

            if self.distanceControl:
                self.targetXYZ[1] = self.cv_data["gate"]["distance"]
                self.currXYZ[1] = 0.0

            if not self.stopped(
                self.cv_data["gate"]["bearing"],
                self.cv_data["gate"]["lateral"],
                self.cv_data["gate"]["distance"],
            ):
                angular_acc, linear_acc = self.correctVehicle(
                    self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ
                )
                self.logger.info(
                    f"currRPY {self.currRPY} targetRPY {self.targetRPY} currXYZ {self.currXYZ} targetXYZ {self.targetXYZ}"
                )
                self.logger.info(f"Angular acc: {angular_acc}")

            else:
                self.logger.info("COMPLETED MOVEMENT TO GATE")


class DepthControlTest(Node):
    def __init__(self, targetXYZ=targetXYZ, targetRPY=targetRPY, testRPYControl=False):
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

        self.get_logger().info(f"Curr Depth: {self.currXYZ[2]}")
        self.get_logger().info(f"Target Depth: {targetXYZ[2]}")
        self.get_logger().info(f"Correction: {linearAcc}")

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
