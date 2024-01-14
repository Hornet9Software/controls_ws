import math
import threading
import time
import rclpy
import controls_core.utilities as utilities

from controls_core.params import *
from controls_core.PID import PIDTuner

from custom_msgs.msg import Correction

from tasks.task import Task

"""
CONVENTIONS

Vehicle forward is along +y.

Any roll/pitch/yaw variable refers to the usual sense of RPY with respect to the vehicle's DOFs.
Any angular xyz variable refers to euler angles w.r.t. the coordinate frame.

For clarity,
roll = angular_position.y
pitch = angular_position.x
"""


class DiveToDepth(Task):
    def __init__(self, targetDepth, tolerance=0.1, setYaw=None):
        super().__init__(task_name="dive_to_depth", outcomes=["done"])

        self.task_state.create_rate(100)
        rclpy.spin_once(self.task_state)

        self.targetDepth = targetDepth

        self.setRoll = 0.0

        if not setYaw:
            self.setYaw = self.state.angular_position.z
        else:
            self.setYaw = setYaw

        self.targetRPY = [self.setRoll, 0.0, self.setYaw]
        self.targetXYZ = [0.0, 0.0, self.targetDepth]

        self.prevDepth = 0.0
        self.prevTime = time.time()

        self.tolerance = tolerance

    def stopped_at_position(self, currDepth):
        currTime = time.time()
        vz = (currDepth - self.prevDepth) / (currTime - self.prevTime)
        self.prevDepth = currDepth
        self.prevTime = currTime

        return (math.fabs(currDepth - self.targetDepth) <= self.tolerance) and (
            math.fabs(vz) <= self.tolerance
        )

    def execute(self, ud):
        print("INITIALISING DIVE TO DEPTH", self.targetDepth)
        print()
        # threading.Thread(
        #     target=PIDTuner, args=[self.positionControl.distancePID], daemon=True
        # ).start()

        return super().execute(ud)

    def run(self, ud):
        while True:
            rclpy.spin_once(self.task_state)

            while (self.state is None) or (self.depth is None):
                rclpy.spin_once(self.task_state)

            self.currRPY = [
                self.state.angular_position.y,
                self.state.angular_position.x,
                self.state.angular_position.z,
            ]

            self.currXYZ = [0.0, 0.0, self.depth]

            if not self.stopped_at_position(self.currXYZ[2]):
                print("CURRENT DEPTH", self.depth)

                corr = Correction()

                corr.target_rpy.data = self.targetRPY
                corr.target_xyz.data = self.targetXYZ
                corr.curr_rpy.data = self.currRPY
                corr.curr_xyz.data = self.currXYZ

                self.publish_correction(corr)
            else:
                print("COMPLETED DIVE TO DEPTH", self.targetDepth, "\n\n=========\n")
                return "done"


class RotateToYaw(Task):
    def __init__(self, targetYaw, tolerance=0.1, setDepth=None):
        super().__init__(task_name="rotate_to_yaw", outcomes=["done"])

        self.task_state.create_rate(100)
        rclpy.spin_once(self.task_state)

        self.targetYaw = targetYaw

        self.setRoll = 0.0

        if not setDepth:
            self.setDepth = self.depth
        else:
            self.setDepth = setDepth

        self.targetRPY = [self.setRoll, 0.0, self.targetYaw]
        self.targetXYZ = [0.0, 0.0, self.setDepth]

        self.prevYaw = 0.0
        self.prevTime = time.time()

        self.tolerance = tolerance

    def stopped_at_bearing(self, currYaw):
        currTime = time.time()
        omega_z = (currYaw - self.prevYaw) / (currTime - self.prevTime)
        self.prevYaw = currYaw
        self.prevTime = currTime

        return (math.fabs(currYaw - self.targetYaw) <= self.tolerance) and (
            math.fabs(omega_z) <= self.tolerance
        )

    def execute(self, ud):
        print("INITIALISING ROTATION TO YAW", self.targetYaw)
        print()

        return super().execute(ud)

    def run(self, ud):
        while True:
            rclpy.spin_once(self.task_state)

            while self.state is None:
                rclpy.spin_once(self.task_state)

            self.currRPY = [
                self.state.angular_position.y,
                self.state.angular_position.x,
                self.state.angular_position.z,
            ]

            self.currXYZ = [0.0, 0.0, self.depth]

            if not self.stopped_at_bearing(self.currRPY[2]):
                print("CURRENT YAW", self.currRPY[2])

                corr = Correction()

                corr.target_rpy.data = self.targetRPY
                corr.target_xyz.data = self.targetXYZ
                corr.curr_rpy.data = self.currRPY
                corr.curr_xyz.data = self.currXYZ

                self.publish_correction(corr)
            else:
                print(
                    "COMPLETED ROTATION TO YAW",
                    self.targetYaw,
                    "\n\n=========\n",
                )
                return "done"


class MoveStraightForTime(Task):
    def __init__(self, timeToMove, setDepth=None, setYaw=None):
        super().__init__(task_name="move_straight_for_time", outcomes=["done"])

        self.task_state.create_rate(100)
        rclpy.spin_once(self.task_state)

        self.timeToMove = timeToMove

        if not setDepth:
            self.setDepth = self.depth
        else:
            self.setDepth = setDepth

        self.setRoll = 0.0

        if not setYaw:
            self.setYaw = self.state.angular_position.z
        else:
            self.setYaw = setYaw

        self.targetRPY = [self.setRoll, 0.0, self.setYaw]
        self.targetXYZ = [0.0, 1.0, self.setDepth]

    def execute(self, ud):
        print("INITIALISING FORWARD MOVEMENT FOR", self.timeToMove, "SECONDS")
        print()

        return super().execute(ud)

    def run(self, ud):
        startingTime = time.time()

        while True:
            rclpy.spin_once(self.task_state)

            while self.state is None:
                rclpy.spin_once(self.task_state)

            self.currRPY = [
                self.state.angular_position.y,
                self.state.angular_position.x,
                self.state.angular_position.z,
            ]

            self.currXYZ = [0.0, 0.0, self.depth]
            currTime = time.time()

            if (currTime - startingTime) < self.timeToMove:
                print("THRUSTING FOR", currTime - startingTime, "MORE SECONDS")

                corr = Correction()

                corr.targetRPY.data = self.targetRPY
                corr.targetXYZ.data = self.targetXYZ
                corr.currRPY.data = self.currRPY
                corr.currXYZ.data = self.currXYZ

                self.publish_correction(corr)
            else:
                print(
                    "COMPLETED FORWARD MOVEMENT FOR",
                    self.timeToMove,
                    "SECONDS",
                    "\n\n=========\n",
                )
                return "done"


class MoveToGate(Task):
    def __init__(
        self,
        tolerance=0.1,
        bearingControl=True,
        lateralControl=True,
        lateralDirection="right",
        distanceControl=True,
        setDepth=-1.0,
    ):
        super().__init__(task_name="rotate_to_yaw", outcomes=["done"])

        self.task_state.create_rate(100)
        rclpy.spin_once(self.task_state)

        self.bearingControl = bearingControl
        self.lateralControl = lateralControl
        self.distanceControl = distanceControl

        self.lateralDirection = 1 if lateralDirection == "right" else -1

        self.prevBearing = self.cv_data["gate"]["bearing"]
        self.prevLateral = self.cv_data["gate"]["lateral"]
        self.prevDistance = self.cv_data["gate"]["distance"]
        self.prevTime = time.time()

        self.setDepth = -1.0
        self.tolerance = tolerance

    def stopped(self, currBearing, currLateral, currDistance):
        currTime = time.time()

        bearingPrime = (currBearing - self.prevBearing) / (currTime - self.prevTime)
        lateralPrime = (currLateral - self.prevLateral) / (currTime - self.prevTime)
        distancePrime = (currDistance - self.prevDistance) / (currTime - self.prevTime)

        self.prevBearing = currBearing
        self.prevLateral = currLateral
        self.prevDistance = currDistance
        self.prevTime = currTime

        bearingStopped = (
            math.fabs(currBearing - (math.pi / 2.0)) <= self.tolerance
        ) and (math.fabs(bearingPrime) <= self.tolerance)
        lateralStopped = (math.fabs(currLateral) <= self.tolerance) and (
            math.fabs(lateralPrime) <= self.tolerance
        )
        distanceStopped = (math.fabs(currDistance) <= self.tolerance) and (
            math.fabs(distancePrime) <= self.tolerance
        )

        return (
            (not self.bearingControl or bearingStopped)
            and (not self.lateralControl or lateralStopped)
            and (not self.distanceControl or distanceStopped)
        )

    def execute(self, ud):
        return super().execute(ud)

    def run(self, ud):
        # BEARING | target is bearing; current is pi/2
        # LATERAL | setpoint is 0; current is lateral
        # DISTANCE | setpoint is distance, current is 0
        while True:
            rclpy.spin_once(self.task_state)

            while self.state is None:
                rclpy.spin_once(self.task_state)

            self.targetRPY = [0.0, 0.0, 0.0]
            self.targetXYZ = [0.0, 0.0, self.setDepth]
            self.currRPY = [self.state.angular_position.y, 0.0, 0.0]
            self.currXYZ = [0.0, 0.0, self.depth]

            if self.bearingControl:
                self.targetRPY[2] = self.cv_data["gate"]["bearing"]
                self.currRPY[2] = math.pi / 2.0

            if self.lateralControl:
                self.targetXYZ[0] = 0.0
                self.currXYZ[0] = (
                    self.lateralDirection * self.cv_data["gate"]["lateral"]
                )

            if self.distanceControl:
                self.targetXYZ[1] = self.cv_data["gate"]["distance"]
                self.currXYZ[1] = 0.0

            if not self.stopped(
                self.cv_data["gate"]["bearing"],
                self.cv_data["gate"]["lateral"],
                self.cv_data["gate"]["distance"],
            ):
                corr = Correction()

                corr.target_rpy.data = self.targetRPY
                corr.target_xyz.data = self.targetXYZ
                corr.curr_rpy.data = self.currRPY
                corr.curr_xyz.data = self.currXYZ

                self.publish_correction(corr)
            else:
                print(
                    "COMPLETED MOVEMENT TO GATE",
                    "\n\n=========\n",
                )
                return "done"
