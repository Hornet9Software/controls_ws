import logging

import numpy as np
from controls_core.attitude_control import AttitudeControl
from controls_core.params import *
from controls_core.position_control import PositionControl
from controls_core.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl


class BasePIDManager:
    def __init__(self, r_PID, p_PID, yaw_PID, x_PID, y_PID, z_PID):
        self.thrusterControl = ThrusterControl()
        self.thrustAllocator = ThrustAllocator()

        self.attitudeControl = AttitudeControl(
            rollPID=r_PID, pitchPID=p_PID, yawPID=yaw_PID
        )
        self.positionControl = PositionControl(
            distancePID=y_PID, lateralPID=x_PID, depthPID=z_PID
        )

    def correctVehicle(
        self, currRPY, targetRPY, currXYZ, targetXYZ, override_forward_acceleration=None
    ):
        angularAcc = self.attitudeControl.getAttitudeCorrection(
            currRPY=currRPY, targetRPY=targetRPY
        )
        linearAcc = self.positionControl.getPositionCorrection(
            currXYZ=currXYZ, targetXYZ=targetXYZ
        )

        if override_forward_acceleration is not None:
            linearAcc[1] = override_forward_acceleration

        thrustValues = self.thrustAllocator.getThrustPWMs(linearAcc, angularAcc)
        logging.info("CORRECTNG WITH THRUST: " + str(thrustValues))
        self.thrusterControl.setThrusters(thrustValues=thrustValues)

        return angularAcc, linearAcc


class GatePIDManager(BasePIDManager):
    def __init__(self):
        super().__init__(rollPID, pitchPID, gatePID, lateralPID, distancePID, depthPID)


class FlarePIDManager(BasePIDManager):
    def __init__(self):
        super().__init__(rollPID, pitchPID, flarePID, lateralPID, distancePID, depthPID)


class NormalPIDManager(BasePIDManager):
    def __init__(self):
        super().__init__(rollPID, pitchPID, yawPID, lateralPID, distancePID, depthPID)
