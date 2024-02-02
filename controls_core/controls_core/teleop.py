import numpy as np
import rclpy
from controls_core.attitude_control import AttitudeControl
from controls_core.params import *
from controls_core.position_control import PositionControl
from controls_core.thruster_allocator import ThrustAllocator
from imu_msg.msg import Imu
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()
attitudeControl = AttitudeControl(rollPID, pitchPID, yawPID)
positionControl = PositionControl(distancePID, lateralPID, depthPID)

class Teleop(Node):
    TELEOP_DEPTH_YAW_TOPIC = "/controls/teleop/depth_yaw"
    TELEOP_ROLL_PITCH_TOPIC = "/controls/teleop/roll_pitch"


    def __init__(self):
        super().__init__("teleop")

        self.create_timer(0.1, self._controlLoop)

        self.currXYZ = [0.0, 0.0, 0.0]
        self.currRPY = [0.0, 0.0, 0.0]

        self.depthListener = self.create_subscription(
            Float32, "/sensors/depth", self._depthProcessing, 10
        )

        self.imuListener = self.create_subscription(
            Imu, "/sensors/imu", self._imuProcessing, 10
        )

        self.depthYawListener = self.create_subscription(
            Twist, self.TELEOP_DEPTH_YAW_TOPIC, self._depthYawProcessing, 10
        )

        self.rollPitchListener = self.create_subscription(
            Twist, self.TELEOP_ROLL_PITCH_TOPIC, self._rollPitchProcessing, 10
        )

        self.depthCorrection = 0.0
        self.yawCorrection = 0.0
        self.rollCorrection = 0.0
        self.pitchCorrection = 0.0

    def _imuProcessing(self, msg: Imu):
        self.currRPY = [
            msg.roll_pitch_yaw.x,
            msg.roll_pitch_yaw.y,
            msg.roll_pitch_yaw.z,
        ]

    def _depthProcessing(self, msg: Float32):
        self.currXYZ[2] = msg.data

    def _depthYawProcessing(self, msg: Twist):
        self.depthCorrection = msg.linear.y
        self.yawCorrection = msg.angular.z

    def _rollPitchProcessing(self, msg: Twist):
        self.rollCorrection = msg.angular.x
        self.pitchCorrection = msg.angular.y
    
    def _controlLoop(self):
        self.targetXYZ = self.currXYZ
        self.targetXYZ[2] += self.depthCorrection

        self.targetRPY =  self.currRPY
        self.targetRPY[0] += self.rollCorrection
        self.targetRPY[1] += self.pitchCorrection
        self.targetRPY[2] += self.yawCorrection

        # self.depthCorrection = 0.0
        # self.yawCorrection = 0.0
        # self.rollCorrection = 0.0
        # self.pitchCorrection = 0.0
        
        angularAcc = attitudeControl.getAttitudeCorrection(
            currRPY=self.currRPY, targetRPY=self.targetRPY
        )
        self.get_logger().info(f"PID angular acceleration: {angularAcc}")

        linearAcc = positionControl.getPositionCorrection(
            currXYZ=self.currXYZ, targetXYZ=self.targetXYZ
        )

        self.get_logger().info(f"Curr Depth: {self.currXYZ[2]}")
        self.get_logger().info(f"Target Depth: {self.targetXYZ[2]}")
        self.get_logger().info(f"Correction: {linearAcc}")

        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs(linearAcc, angularAcc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()

    try:
        rclpy.spin(teleop)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()
