import rclpy

from rclpy.node import Node

from custom_msgs import Correction

from controls_core.attitude_control import AttitudeControl
from controls_core.position_control import PositionControl
from controls_core.thruster_allocator import ThrustAllocator
from controls_core.params import *

from thrusters.thrusters import ThrusterControl


class PIDManager(Node):
    CORRECTION_TOPIC = "/controls/correction"

    def __init__(self):
        super().__init__("pid_manager")

        self.correctionListener = self.create_subscription(
            Correction, self.CORRECTION_TOPIC, self._onReceiveCorrection, 10
        )

        self.thrusterControl = ThrusterControl()
        self.thrustAllocator = ThrustAllocator()

        self.attitudeControl = AttitudeControl(rollPID=rollPID, yawPID=yawPID)
        self.positionControl = PositionControl(
            distancePID=distancePID, lateralPID=lateralPID, depthPID=depthPID
        )

    def _onReceiveCorrection(self, msg):
        currRPY = msg.currRPY.data
        targetRPY = msg.targetRPY.data
        currXYZ = msg.currXYZ.data
        targetXYZ = msg.targetXYZ.data

        angularAcc = self.attitudeControl.getAttitudeCorrection(
            currRPY=currRPY, targetRPY=targetRPY
        )
        linearAcc = self.positionControl.getPositionCorrection(
            currXYZ=currXYZ, targetXYZ=targetXYZ
        )

        thrustValues = self.thrustAllocator.getThrustPWMs(linearAcc, angularAcc)
        print("CORRECTNG WITH THRUST", thrustValues)
        print("\n=========\n")
        self.thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args=None):
    rclpy.init(args=args)
    pidManager = PIDManager()

    try:
        rclpy.spin(pidManager)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
