import numpy as np

imuZero = [0.0, 0.0, 2.3114576746942372]


class AttitudeControl:
    def __init__(self, rollPID, yawPID):
        self.rollPID = rollPID
        self.yawPID = yawPID

    def boundAngle(self, angle):
        """
        Bound angle to [-pi, pi]
        """
        angle = angle % (2 * np.pi)
        if angle < -np.pi:
            return 2 * np.pi + angle
        elif angle > np.pi:
            return angle - 2 * np.pi
        else:
            return angle

    def correctIMU(self, currAtt):
        """
        Correct IMU by subtracting imuZero.
        """
        corrAtt = []
        for att, zero in zip(currAtt, imuZero):
            corrAtt.append(self.boundAngle(att - zero))

        return corrAtt

    def getAngleError(self, currAngle, targetAngle):
        """
        If target = pi/2 and curr = pi,
            Unbounded error is pi/2.
            Bounded error is pi/2.
            Curr is pi/2 bigger than target.
            Rotate clockwise.

        If target = -pi/2 and curr = pi,
            Unbounded error is 3*pi/2.
            Bounded error is -pi/2.
            Curr is pi/2 smaller than target.
            Rotate anti-clockwise.

        If target = -pi/2 and curr = 0,
            Unbounded error is -pi/2.
            Bounded error is -pi/2.
            Curr is pi/2 smaller than target.
            Rotate anti-clockwise.
        """
        # Might not need this
        currAngle = self.boundAngle(currAngle)
        targetAngle = self.boundAngle(targetAngle)

        return self.boundAngle(currAngle - targetAngle)

    def getAttitudeCorrection(self, currAttRPY, targetAttRPY):
        currRoll, _, currYaw = self.correctIMU(currAttRPY)
        targetRoll, _, targetYaw = targetAttRPY

        print("Curr Yaw", currYaw)
        print("Target Yaw", targetYaw)
        print(
            "Angle Error", self.getAngleError(currAngle=currYaw, targetAngle=targetYaw)
        )

        rollAcc = self.rollPID.compute(
            setpoint=0.0,
            process_variable=self.getAngleError(
                currAngle=currRoll, targetAngle=targetRoll
            ),
        )
        yawAcc = self.yawPID.compute(
            setpoint=0.0,
            process_variable=self.getAngleError(
                currAngle=currYaw, targetAngle=targetYaw
            ),
        )
        angular_acc = [rollAcc, 0, yawAcc]

        return angular_acc

    def getSteer(self, deviationRPY):
        return self.getAttitudeCorrection([0, 0, 0], deviationRPY)