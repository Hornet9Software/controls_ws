import math
import time

import rclpy
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

        if setYaw is not None:
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
        self.logger.info("INITIALISING DIVE TO DEPTH {}".format(self.targetDepth))
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
                self.logger.info("CURRENT DEPTH: {}".format(self.depth))

                self.correctVehicle(
                    self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ
                )
            else:
                self.logger.info("COMPLETED DIVE TO DEPTH {}".format(self.targetDepth))
                return "done"


class RotateToYaw(Task):
    def __init__(self, targetYaw, tolerance=0.02, setDepth=None):
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

            while (self.state is None) or (self.depth is None):
                rclpy.spin_once(self.task_state)

            self.currRPY = [
                self.state.angular_position.y,
                self.state.angular_position.x,
                self.state.angular_position.z,
            ]

            self.currXYZ = [0.0, 0.0, self.depth]

            if not self.stopped_at_bearing(self.currRPY[2]):
                print("CURRENT YAW", self.currRPY[2])

                self.correctVehicle(
                    self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ
                )
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
        self.logger.info(
            "INITIALISING FORWARD MOVEMENT FOR " + str(self.timeToMove) + " SECONDS"
        )

        return super().execute(ud)

    def run(self, ud):
        startingTime = time.time()

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
            currTime = time.time()

            if (currTime - startingTime) < self.timeToMove:
                self.logger.info(
                    "THRUSTING FOR "
                    + str(self.timeToMove - (currTime - startingTime))
                    + " MORE SECONDS"
                )

                self.correctVehicle(
                    self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ
                )
            else:
                self.logger.info(
                    "COMPLETED FORWARD MOVEMENT FOR "
                    + str(self.timeToMove)
                    + " SECONDS"
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

        self.prevBearing = 0.0
        self.prevLateral = 0.0
        self.prevDistance = 0.0
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
        self.logger.info("INITIALISING MOVEMENT TO GATE")
        return super().execute(ud)

    def run(self, ud):
        # BEARING | target is bearing; current is pi/2
        # LATERAL | setpoint is 0; current is lateral
        # DISTANCE | setpoint is distance, current is 0
        while True:
            rclpy.spin_once(self.task_state)

            # self.logger.info("hello1")
            while (
                (self.state is None)
                or (self.depth is None)
                or (self.cv_data["gate"] is None)
            ):
                rclpy.spin_once(self.task_state)

            # self.logger.info("hello123")
            self.targetRPY = [0.0, 0.0, 0.0]
            self.targetXYZ = [0.0, 0.0, self.setDepth]
            self.currRPY = [self.state.angular_position.y, 0.0, 0.0]
            self.currXYZ = [0.0, 0.0, self.depth]

            if self.bearingControl:
                self.targetRPY[2] = (
                    self.cv_data["gate"]["bearing"] + self.state.angular_position.z
                )
                self.currRPY[2] = self.state.angular_position.z

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
                self.correctVehicle(
                    self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ
                )
            else:
                self.logger.info("COMPLETED MOVEMENT TO GATE")
                return "done"


class HoldCurrentState(Task):
    def __init__(self, setPosition=None, setYaw=None, setRoll=None):
        super().__init__(task_name="hold_current_state")
        self.task_state.create_rate(100)
        rclpy.spin_once(self.task_state)

        # Use provided position, otherwise take current position
        if not setPosition:
            self.setPosition = [
                self.linear_position.x,
                self.linear_position.y,
                self.depth
            ]
        else:
            self.setPosition = setPosition

        if not setRoll:
            self.setRoll = self.state.angular_position.y
            # self.setRoll = 0.0
        else:
            self.setRoll = setRoll

        if not setYaw:
            self.setYaw = self.state.angular_position.z
        else:
            self.setYaw = setYaw

        # pitch is 0.0
        self.targetRPY = [self.setRoll, 0.0, self.setYaw]
        self.targetXYZ = self.setPosition



    def execute(self, ud):
        self.logger.info("INITIALISING HOLD CURRENT STATE AT {} {}".format(self.targetXYZ, self.targetRPY))
        return super().execute(ud)

    def run(self, ud):
        while True:
            rclpy.spin_once(self.task_state)

            self.currXYZ = [
                self.state.linear_position.x,
                self.state.linear_position.y,
                self.depth
            ]

            self.currRPY = [
                self.state.angular_position.y, 0.0, self.state.angular_position.z
            ]
            self.logger.info("CURRENT STATE {} {} TO HOLD AT {} {}".format(self.currXYZ, self.currRPY, self.targetXYZ, self.targetRPY))
            

         
            self.correctVehicle(self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ)
