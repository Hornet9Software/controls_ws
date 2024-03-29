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
targetRPY = np.array([np.radians(-5), 0, np.radians(70)])

class QualificationSequence(Node):
    def __init__(self, targetXYZ=targetXYZ, targetRPY=targetRPY):
        super().__init__("qualification_sequence")

        self.targetXYZ = targetXYZ
        self.targetRPY = targetRPY

        self.create_timer(0.1, self._controlLoop)
        self.first = True

        self.currXYZ = [0.0, 0.0, 0.0]
        self.currRPY = [0.0, 0.0, 0.0]

        self.depthListener = self.create_subscription(
            Float32, "/sensors/depth", self._depthProcessing, 10
        )

        self.imuListener = self.create_subscription(
            Imu, "/sensors/imu/corrected", self._imuProcessing, 10
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

        self.currRPY = attitudeControl.correctIMU(self.currRPY)
        angularAcc = attitudeControl.getAttitudeCorrection(
            currRPY=self.currRPY, targetRPY=self.targetRPY
        )
        self.get_logger().info(f"PID angular acceleration: {angularAcc}")

        linearAcc = positionControl.getPositionCorrection(
            currXYZ=self.currXYZ, targetXYZ=targetXYZ
        )

        curr_time = time.time()
        delta_t = curr_time - self.init_time

        eqm_time = 7.0

# 1 METRE 
#        forward_time = 2.0
#        braking_time = 0.0
#        forward_acceleration = 1.0
#        braking_acceleration = 0.0

# 2 METRES
#        forward_time = 2.5
#        braking_time = 0.0
#        forward_acceleration = 3.0
#        braking_acceleration = 0.0

# 3 METRES
#        forward_time = 4
#        braking_time = 0.0
#        forward_acceleration = 3.0
#        braking_acceleration = 0.0

# 4 METRES
#        forward_time = 5.5
#        braking_time = 0.0
#        forward_acceleration = 3.0
#        braking_acceleration = 0.0

# 5 METRES
#        forward_time = 7.5
#        braking_time = 0.0
#        forward_acceleration = 3.0
#        braking_acceleration = 0.0

# 6 METRES
#        forward_time = 8.8
#        braking_time = 0.0
#        forward_acceleration = 3.0
#        braking_acceleration = 0.0

# 7 METRES
#        forward_time = 10
#        braking_time = 0.0
#        forward_acceleration = 3.0
#        braking_acceleration = 0.0

# 8 METRES
#        forward_time = 12.5
#        braking_time = 0.0
#        forward_acceleration = 3.0
#        braking_acceleration = 0.0

# 9 METRES
#        forward_time = 14
#        braking_time = 0.0
#        forward_acceleration = 3.0
#        braking_acceleration = 0.0

# 10 METRES
#        forward_time = 16
#        braking_time = 0.0
#        forward_acceleration = 3.0
#        braking_acceleration = 0.0


        distance_to_move = 14
        forward_time = 1.675 * distance_to_move - 1.0722
        braking_time = 0.0
        forward_acceleration = 3.0
        braking_acceleration = 0.0

        if delta_t < eqm_time:
            linearAcc[1] = 0.0
        elif delta_t < eqm_time + forward_time:
            linearAcc[1] = -forward_acceleration
            self.get_logger().info("MOVING FORWARD")
        elif delta_t >= eqm_time + forward_time and delta_t < eqm_time + forward_time + braking_time:
            self.get_logger().info("BRAKING")
            linearAcc[1] = braking_acceleration
        else:
            linearAcc[1] = -0.0

        self.get_logger().info(f"Correction: {linearAcc}")

        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs(linearAcc, angularAcc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args=None):
    rclpy.init(args=args)
    quali = QualificationSequence()

    try:
        rclpy.spin(quali)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()
