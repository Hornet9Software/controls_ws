import numpy as np


class PositionControl:
    def __init__(self, distancePID, lateralPID, depthPID):
        self.distancePID = distancePID
        self.lateralPID = lateralPID
        self.depthPID = depthPID

    def getPositionCorrection(self, currXYZ, targetXYZ):
        currLateral, currDistance, currDepth = currXYZ
        targetLateral, targetDistance, targetDepth = targetXYZ

        yAcc = self.distancePID.compute(
            setpoint=targetDistance,
            process_variable=currDistance,
        )

        xAcc = self.lateralPID.compute(
            setpoint=targetLateral, process_variable=currLateral
        )

        zAcc = self.depthPID.compute(setpoint=targetDepth, process_variable=currDepth)

        linAcc = [xAcc, yAcc, zAcc]

        return linAcc
