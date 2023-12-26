#!/usr/bin/env python3

import numpy as np


class DistanceControl:
    def __init__(self, distancePID):
        self.distancePID = distancePID

    def getDistanceCorrection(self, currDistance):
        print("Current Distance Deviation", currDistance)

        yAcc = self.distancePID.compute(
            setpoint=0.0,
            process_variable=currDistance,
        )

        linAcc = [0.0, yAcc, 0.0]

        return linAcc
