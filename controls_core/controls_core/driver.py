import numpy as np
import rclpy
from controls_core.thruster_allocator import ThrustAllocator
from controls_core.utilities import quat_to_list
from nav_msgs.msg import Odometry

# from PID import PID
from rclpy.node import Node
from simple_pid import PID
from tf_transformations import euler_from_quaternion
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()

imuZero = [-0.0, -0.0, 2.3114576746942372]


class AttitudeControl:
    def __init__(self):
        self.thrustAllocator = ThrustAllocator()
        self.rollPID = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0.0, sample_time=None)
        self.yawPID = PID(Kp=10.0, Ki=0.0, Kd=0.0, setpoint=0.0, sample_time=None)

    def correctIMU(self, currAtt):
        correctedVals = []
        for val, zero in zip(currAtt, imuZero):
            corrected = val - zero
            if corrected < -np.pi:
                corrected = 2*np.pi + corrected
            correctedVals.append(corrected)
        return correctedVals

    def getAttitudeCorrection(self, currAttRPY, targetAttRPY):
        currRoll, _, currYaw = self.correctIMU(currAttRPY)
        targetRoll, _, targetYaw = targetAttRPY

        self.rollPID.setpoint = targetRoll
        self.yawPID.setpoint = targetYaw

        rollAcc = self.rollPID(currRoll)
        yawAcc = self.yawPID(currYaw)
        angular_acc = [rollAcc, 0, yawAcc]

        return angular_acc


class Driver(Node):
    def __init__(self) -> None:
        super().__init__("driver_node")
        self.state_subscriber = self.create_subscription(
            Odometry, "/state", self.attitudeControl, 10
        )
        self.attitudeControl = AttitudeControl()

        self.linear_acc = np.array([0, 0, 0])
        self.angular_acc = np.array([0, 0, 0])

    def _drive(self, msg: Odometry):
        currAttQuat = msg.pose.pose.orientation
        currAttRPY = euler_from_quaternion(quat_to_list(currAttQuat))
        attCorr = self.attitudeControl.getAttitudeCorrection(
            currAttRPY=currAttRPY, targetAttRPY=[0, 0, 0]
        )

        thrustValues = thrustAllocator.getThrustPWMs(
            self.linear_acc, self.angular_acc + np.array(attCorr)
        )
        thrusterControl.setThrusters(thrustValues=thrustValues)

    def drive(self, linear_acc, angular_acc):
        self.linear_acc = linear_acc
        self.angular_acc = angular_acc

    def kill(self):
        self.linear_acc = np.array([0, 0, 0])
        self.angular_acc = np.array([0, 0, 0])

