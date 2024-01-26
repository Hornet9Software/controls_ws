import logging

import numpy as np
from controls_core.attitude_control import AttitudeControl
from controls_core.params import *
from controls_core.position_control import PositionControl
from controls_core.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl


class PIDManager:
    def __init__(self):
        self.thrusterControl = ThrusterControl()
        self.thrustAllocator = ThrustAllocator()

        self.attitudeControl = AttitudeControl(
            rollPID=rollPID, pitchPID=pitchPID, yawPID=yawPID_camera
        )
        self.positionControl = PositionControl(
            distancePID=distancePID, lateralPID=lateralPID, depthPID=depthPID
        )

    def correctVehicle(self, currRPY, targetRPY, currXYZ, targetXYZ):
        angularAcc = self.attitudeControl.getAttitudeCorrection(
            currRPY=currRPY, targetRPY=targetRPY
        )
        linearAcc = self.positionControl.getPositionCorrection(
            currXYZ=currXYZ, targetXYZ=targetXYZ
        )

        thrustValues = self.thrustAllocator.getThrustPWMs(linearAcc, angularAcc)
        logging.info("CORRECTNG WITH THRUST: " + str(thrustValues))
        self.thrusterControl.setThrusters(thrustValues=thrustValues)

        return angularAcc, linearAcc
