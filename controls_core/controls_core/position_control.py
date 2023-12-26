#!/usr/bin/env python3

import numpy as np


class PositionControl:
    def __init__(self, distancePID, lateralPID, depthPID):
        self.distancePID = distancePID
        self.lateralPID = lateralPID
        self.depthPID = depthPID

    def getPositonCorrection(
        self,
        currDistance,
        desiredDistance,
        currLateral,
        desiredLateral,
        currDepth,
        desiredDepth,
    ):
        print("Current Distance Deviation", currDistance - desiredDistance)
        print("Current Lateral Deviation", currLateral - desiredLateral)
        print("Current Depth Deviation", currDepth - desiredDepth)

        yAcc = self.distancePID.compute(
            setpoint=desiredDistance,
            process_variable=currDistance,
        )

        xAcc = self.lateralPID.compute(
            setpoint=desiredLateral, process_variable=currLateral
        )

        zAcc = self.depthPID.compute(setpoint=desiredDepth, process_variable=currDepth)

        linAcc = [xAcc, yAcc, zAcc]

        return linAcc
