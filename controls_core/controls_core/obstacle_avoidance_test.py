import numpy as np
import rclpy
from controls_core.attitude_control import AttitudeControl
from controls_core.params import *
from controls_core.position_control import PositionControl
from controls_core.thruster_allocator import ThrustAllocator
from imu_msg.msg import Imu
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()
attitudeControl = AttitudeControl(rollPID, pitchPID, yawPID)
positionControl = PositionControl(distancePID, lateralPID, depthPID)

targetXYZ = np.array([0, 0, -1.2])
targetRPY = [0, 0, 0]


class ObstacleAvoidanceTest(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_test")

        self.create_timer(0.1, self._controlLoop)

        self.currXYZ = [0.0, 0.0, 0.0]
        self.currRPY = [0.0, 0.0, 0.0]

        self.depthListener = self.create_subscription(
            Float32, "/sensors/depth", self._depthProcessing, 10
        )

        self.imuListener = self.create_subscription(
            Imu, "/sensors/imu/corrected", self._imuProcessing, 10
        )

        self.gate_listener = self.create_subscription(
            Float32MultiArray,
            "/object/orange-flare/bearing_lateral_distance",
            lambda msg: self._on_receive_cv_data(msg, "gate"),
            10,
        )

        self.cv_data = {"gate": {}}

    def _on_receive_cv_data(self, msg: Float32MultiArray, objectName):
        msgData = np.array(msg.data).tolist()
        self.cv_data[objectName]["bearing"] = msgData[0]
        self.cv_data[objectName]["lateral"] = msgData[1]
        self.cv_data[objectName]["distance"] = msgData[2]

        self.targetRPY[2] = msgData[0]
        self.get_logger().info(self.cv_data.__repr__())
        self.get_logger().info(f"Target RPY changed to {self.targetRPY}")

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
            self.currRPY = attitudeControl.correctIMU(self.currRPY)
            self.currRPY[2] = 0.0
            angularAcc = attitudeControl.getAttitudeCorrection(
                currRPY=self.currRPY, targetRPY=self.targetRPY
            )
            self.get_logger().info(f"PID angular acceleration: {angularAcc}")
        else:
            angularAcc = [0.0, 0.0, 0.0]

        linearAcc = positionControl.getPositionCorrection(
            currXYZ=self.currXYZ, targetXYZ=targetXYZ
        )

        if (
            abs(attitudeControl.getAngleError(self.currRPY[2], self.targetRPY[2]))
            < 0.10
            or self.cv_data["gate"]["distance"] < 1
        ):
            linearAcc[1] = 2.0
        else:
            linearAcc[1] = 0.0

        # self.get_logger().info(f"Curr Depth: {self.currXYZ[2]}")
        # self.get_logger().info(f"Target Depth: {targetXYZ[2]}")
        self.get_logger().info(
            f"Target RPY: {self.targetRPY}, Curr RPY: {self.currRPY}"
        )
        self.get_logger().info(f"Correction: {linearAcc}")

        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs(linearAcc, angularAcc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args=None):
    rclpy.init(args=args)
    depthTest = RotateToGateTest(testRPYControl=True)

    try:
        rclpy.spin(depthTest)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()
