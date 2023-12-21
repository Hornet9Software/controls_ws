import numpy as np
from controls_core.attitude_control import AttitudeControl
from controls_core.thruster_allocator import ThrustAllocator
from controls_core.utilities import quat_to_list
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()


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
