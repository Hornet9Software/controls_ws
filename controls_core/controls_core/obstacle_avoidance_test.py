import math

import numpy as np
import rclpy
from imu_msg.msg import Imu
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from thrusters.thrusters import ThrusterControl

from controls_core.attitude_control import AttitudeControl
from controls_core.params import *
from controls_core.position_control import PositionControl
from controls_core.thruster_allocator import ThrustAllocator

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()
attitudeControl = AttitudeControl(rollPID, pitchPID, yawPID)
positionControl = PositionControl(distancePID, lateralPID, depthPID)


class ObstacleAvoidanceTest(Node):
    def __init__(
        self,
        targetXYZ=[0.0, 0.0, -1.2],
        objectName="orange_flare",
        x_power=-1, # this was -1
        y_power=-1, # this was -1
        x_fac=0.1, #this was 1.0
        y_fac=0.1, # this was 1.0
    ):
        super().__init__("obstacle_avoidance_test")

        self.objectName = objectName
        self.x_power = x_power
        self.y_power = y_power
        self.x_fac = x_fac
        self.y_fac = y_fac

        self.currXYZ = [0.0, 0.0, 0.0]
        self.targetXYZ = targetXYZ

        self.depthListener = self.create_subscription(
            Float32, "/sensors/depth", self._depthProcessing, 10
        )

        # self.imuListener = self.create_subscription(
        #     Imu, "/sensors/imu/corrected", self._imuProcessing, 10
        # )

        self.flare_listener = self.create_subscription(
            Float32MultiArray,
            f"/object/{self.objectName}/bearing_lateral_distance",
            lambda msg: self._on_receive_cv_data(msg, self.objectName),
            10,
        )

        self.cv_data = {
            self.objectName: {"bearing": 0.0, "lateral": 0.0, "distance": 0.0}
        }

        self.create_timer(0.1, self._controlLoop)

    def _on_receive_cv_data(self, msg: Float32MultiArray, objectName):
        msgData = np.array(msg.data).tolist()
        self.cv_data[objectName]["bearing"] = msgData[0]
        self.cv_data[objectName]["lateral"] = msgData[1]
        self.cv_data[objectName]["distance"] = msgData[2]

        self.get_logger().info(self.cv_data.__repr__())

    # def _imuProcessing(self, msg: Imu):
    #     self.currRPY = [
    #         msg.roll_pitch_yaw.x,
    #         msg.roll_pitch_yaw.y,
    #         msg.roll_pitch_yaw.z,
    #     ]

    def _depthProcessing(self, msg: Float32):
        self.currXYZ[2] = msg.data

    def _controlLoop(self):
        angularAcc = [0.0, 0.0, 0.0]

        theta = self.cv_data[self.objectName]["bearing"]
        d = self.cv_data[self.objectName]["distance"]

        sgn_theta = np.sign(theta)
        theta = np.abs(theta)

        x_repulsion = self.x_fac * ((d * np.sin(theta)) ** self.x_power) * sgn_theta
        y_repulsion = self.y_fac * ((d * np.cos(theta)) ** self.y_power) * -1.0

        x_repulsion = 0.0 if math.isnan(x_repulsion) or abs(x_repulsion) == float('inf') else x_repulsion
        y_repulsion = 0.0 if math.isnan(y_repulsion) or abs(y_repulsion) == float('inf') else y_repulsion

        self.targetXYZ[0] = x_repulsion
        self.targetXYZ[1] = y_repulsion

        linearAcc = positionControl.getPositionCorrection(
            currXYZ=self.currXYZ, targetXYZ=self.targetXYZ
        )

        # self.get_logger().info(f"Curr Depth: {self.currXYZ[2]}")
        # self.get_logger().info(f"Target Depth: {targetXYZ[2]}")
        self.get_logger().info(
            f"Target YXZ: {self.targetXYZ}, Curr XYZ: {self.currXYZ}"
        )
        self.get_logger().info(f"Correction: {linearAcc}")

        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs(linearAcc, angularAcc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args=None):
    rclpy.init(args=args)
    obstacleTest = ObstacleAvoidanceTest()

    try:
        rclpy.spin(obstacleTest)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()
