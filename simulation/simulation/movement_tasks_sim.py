import math
import time
import threading

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from tasks.task import Task
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from controls_core.attitude_control import AttitudeControl
from controls_core.position_control import PositionControl
from controls_core.params import *
from controls_core.PID import PIDTuner

from simulation.thrusters_sim import ThrusterControl
from simulation.thrusters_sim import ThrustAllocator

import controls_core.utilities as utilities
from controls_core.utilities import pos_to_list, quat_to_list


class RotateToObject(Task):
    def __init__(self, objectName, tolerance):
        super().__init__(task_name="rotate_to_object", outcomes=["done"])
        self.objectName = objectName
        self.tolerance = tolerance

    def execute(self, ud):
        self.thrusterControl = ThrusterControl()
        self.thrustAllocator = ThrustAllocator()

        self.currentBearing = [0.0, 0.0, 90.0]
        self.linearAcc = [0.0, 0.0, 0.0]

        self.attitudeControl = AttitudeControl(rollPID=rollPID, yawPID=cameraSteerPID)

        return super().execute(ud)

    def stopped_at_bearing(self, currentBearing):
        return math.fabs(currentBearing) <= self.tolerance

    def run(self, ud):
        self.task_state.create_rate(100)

        print("INITIALISING ROTATION TO OBJECT", self.objectName)
        print()

        while True:
            rclpy.spin_once(self.task_state)

            while self.cv_data[self.objectName]["bearing"] is None:
                rclpy.spin_once(self.task_state)

            if not self.stopped_at_bearing(self.cv_data[self.objectName]["bearing"]):
                self.currentBearing[2] = self.cv_data[self.objectName]["bearing"]

                attCorr = self.attitudeControl.getSteer(self.currentBearing)
                print("ATTITUDE CORRECTION TO OBJECT", self.objectName, attCorr)

                thrustValues = self.thrustAllocator.getThrustPWMs(
                    self.linearAcc, attCorr
                )
                self.thrusterControl.setThrusters(thrustValues=thrustValues)
                print("CORRECTNG WITH THRUST", thrustValues)
                print("\n=========\n")
            else:
                print(
                    "COMPLETED ROTATION TO OBJECT",
                    self.objectName,
                    "\n\n=========\n",
                )
                return "done"


class LateralShiftToObject(Task):
    def __init__(self, objectName, tolerance):
        super().__init__(task_name="lateral_shift_to_object", outcomes=["done"])
        self.objectName = objectName
        self.tolerance = tolerance

    def execute(self, ud):
        self.thrusterControl = ThrusterControl()
        self.thrustAllocator = ThrustAllocator()

        self.angularAcc = [0.0, 0.0, 0.0]
        self.linearAcc = [1.0, 0.0, 0.0]
        self.currLateral = 90.0

        self.positionControl = PositionControl(
            distancePID=distancePID, lateralPID=lateralPID, depthPID=depthPID
        )

        return super().execute(ud)

    def stopped_at_position(self, currentLateral):
        return math.fabs(currentLateral) <= self.tolerance

    def run(self, ud):
        self.task_state.create_rate(100)

        print("INITIALISING LATERAL SHIFT TO OBJECT", self.objectName)
        print()

        while True:
            rclpy.spin_once(self.task_state)

            while self.cv_data[self.objectName]["lateral"] is None:
                rclpy.spin_once(self.task_state)

            if not self.stopped_at_position(self.cv_data[self.objectName]["lateral"]):
                self.currLateral = self.cv_data[self.objectName]["lateral"]
                print("LATERAL CORRECTION TO OBJECT", self.objectName, self.currLateral)

                self.linearAcc = self.positionControl.getPositonCorrection(
                    currDistance=0.0,
                    desiredDistance=0.0,
                    currLateral=self.currLateral,
                    desiredLateral=0.0,
                    currDepth=0.0,
                    desiredDepth=0.0,
                )
                thrustValues = self.thrustAllocator.getThrustPWMs(
                    self.linearAcc, self.angularAcc
                )
                print("CORRECTNG WITH THRUST", thrustValues)
                print("\n=========\n")
                self.thrusterControl.setThrusters(thrustValues=thrustValues)
            else:
                print(
                    "COMPLETED LATERAL SHIFT TO OBJECT",
                    self.objectName,
                    "\n\n=========\n",
                )
                return "done"


class AlignToObject(Task):
    def __init__(self, objectName, bearingTolerance, lateralTolerance):
        super().__init__(task_name="align_to_object", outcomes=["done"])
        self.objectName = objectName
        self.bearingTolerance = bearingTolerance
        self.lateralTolerance = lateralTolerance

    def execute(self, ud):
        self.thrusterControl = ThrusterControl()
        self.thrustAllocator = ThrustAllocator()

        self.currentBearing = [0.0, 0.0, 90.0]
        self.linearAcc = [1.0, 0.0, 0.0]
        self.currLateral = 90.0

        self.attitudeControl = AttitudeControl(rollPID=rollPID, yawPID=cameraSteerPID)
        self.positionControl = PositionControl(
            distancePID=distancePID, lateralPID=lateralPID, depthPID=depthPID
        )

        return super().execute(ud)

    def stopped_at_bearing(self, currentBearing):
        return math.fabs(currentBearing) <= self.bearingTolerance

    def stopped_at_position(self, currentLateral):
        return math.fabs(currentLateral) <= self.lateralTolerance

    def run(self, ud):
        self.task_state.create_rate(100)

        print("INITIALISING ALIGN TO OBJECT", self.objectName)
        print()

        while True:
            rclpy.spin_once(self.task_state)

            while (self.cv_data[self.objectName]["bearing"] is None) or (
                self.cv_data[self.objectName]["lateral"] is None
            ):
                rclpy.spin_once(self.task_state)

            if not (
                self.stopped_at_bearing(self.cv_data[self.objectName]["bearing"])
                and self.stopped_at_position(self.cv_data[self.objectName]["lateral"])
            ):
                self.currentBearing[2] = self.cv_data[self.objectName]["bearing"]
                self.currLateral = self.cv_data[self.objectName]["lateral"]

                print("CURRENT BEARING", self.currentBearing[2])
                print("CURRENT LATERAL", self.currLateral)

                self.angularAcc = self.attitudeControl.getSteer(self.currentBearing)
                self.linearAcc = self.positionControl.getPositonCorrection(
                    currDistance=0.0,
                    desiredDistance=0.0,
                    currLateral=self.currLateral,
                    desiredLateral=0.0,
                    currDepth=0.0,
                    desiredDepth=0.0,
                )
                thrustValues = self.thrustAllocator.getThrustPWMs(
                    self.linearAcc, self.angularAcc
                )
                print("CORRECTNG WITH THRUST", thrustValues)
                print("\n=========\n")
                self.thrusterControl.setThrusters(thrustValues=thrustValues)
            else:
                print(
                    "COMPLETED ALIGNMENT TO OBJECT", self.objectName, "\n\n=========\n"
                )
                return "done"


class MoveStraightToObject(Task):
    def __init__(self, objectName, tolerance):
        super().__init__(task_name="move_straight_to_object", outcomes=["done"])
        self.objectName = objectName
        self.tolerance = tolerance

    def execute(self, ud):
        self.thrusterControl = ThrusterControl()
        self.thrustAllocator = ThrustAllocator()

        self.angularAcc = [0.0, 0.0, 0.0]
        self.linearAcc = [0.0, 1.0, 0.0]
        self.currLateral = 0.0

        self.positionControl = PositionControl(
            distancePID=distancePID, lateralPID=lateralPID, depthPID=depthPID
        )

        return super().execute(ud)

    def stopped_at_position(self, currentDistance):
        return math.fabs(currentDistance) <= self.tolerance

    def run(self, ud):
        self.task_state.create_rate(100)

        print("INITIALISING MOVEMENT TO OBJECT", self.objectName)
        print()

        while True:
            rclpy.spin_once(self.task_state)

            while self.cv_data[self.objectName]["distance"] is None:
                rclpy.spin_once(self.task_state)

            if not self.stopped_at_position(self.cv_data[self.objectName]["distance"]):
                self.currDistance = self.cv_data[self.objectName]["distance"]
                print(
                    "DISTANCE CORRECTION TO OBJECT", self.objectName, self.currDistance
                )

                self.linearAcc = self.positionControl.getPositonCorrection(
                    currDistance=self.currDistance,
                    desiredDistance=0.0,
                    currLateral=0.0,
                    desiredLateral=0.0,
                    currDepth=0.0,
                    desiredDepth=0.0,
                )
                thrustValues = self.thrustAllocator.getThrustPWMs(
                    self.linearAcc, self.angularAcc
                )
                print("CORRECTNG WITH THRUST", thrustValues)
                print("\n=========\n")
                self.thrusterControl.setThrusters(thrustValues=thrustValues)
            else:
                print(
                    "COMPLETED MOVEMENT TO OBJECT", self.objectName, "\n\n=========\n"
                )
                return "done"


class DiveToDepth(Task):
    def __init__(self, desiredDepth, tolerance):
        super().__init__(task_name="dive_to_depth", outcomes=["done"])
        self.desiredDepth = desiredDepth
        self.tolerance = tolerance

    def execute(self, ud):
        self.thrusterControl = ThrusterControl()
        self.thrustAllocator = ThrustAllocator()

        self.angularAcc = [0.0, 0.0, 0.0]
        self.linearAcc = [0.0, 0.0, 1.0]
        self.currLateral = 0.0

        self.positionControl = PositionControl(
            distancePID=distancePID, lateralPID=lateralPID, depthPID=depthPID
        )

        threading.Thread(
            target=PIDTuner, args=[self.positionControl.distancePID], daemon=True
        ).start()

        return super().execute(ud)

    def stopped_at_position(self, currentDepth):
        return math.fabs(currentDepth - self.desiredDepth) <= self.tolerance

    def run(self, ud):
        self.task_state.create_rate(100)

        print("INITIALISING DIVE TO DEPTH", self.desiredDepth)
        print()

        while True:
            rclpy.spin_once(self.task_state)

            while self.depth is None:
                rclpy.spin_once(self.task_state)

            if not self.stopped_at_position(self.depth):
                print("CURRENT DEPTH", self.depth)

                self.linearAcc = self.positionControl.getPositonCorrection(
                    currDistance=0.0,
                    desiredDistance=0.0,
                    currLateral=0.0,
                    desiredLateral=0.0,
                    currDepth=self.depth,
                    desiredDepth=self.desiredDepth,
                )
                thrustValues = self.thrustAllocator.getThrustPWMs(
                    self.linearAcc, self.angularAcc
                )
                print("CORRECTNG WITH THRUST", thrustValues)
                print("\n=========\n")
                self.thrusterControl.setThrusters(thrustValues=thrustValues)
            else:
                print("COMPLETED DIVE TO DEPTH", self.desiredDepth, "\n\n=========\n")
                return "done"


# # Note: odom is global base_link is local
# class MoveToPoseGlobalTask(Task):
#     # Move to pose given in global coordinates
#     def __init__(self, x, y, z, roll, pitch, yaw):
#         super().__init__(task_name="move_to_global_task", outcomes=["done"])

#         self.coords = [x, y, z, roll, pitch, yaw]

#     def execute(self, ud):
#         self.initial_state = self.state

#         # pose from userdata if available
#         arg_names = ["x", "y", "z", "roll", "pitch", "yaw"]
#         for i in range(len(arg_names)):
#             if arg_names[i] in ud:
#                 self.coords[i] = ud[arg_names[i]]

#         self.desired_pose = Pose()
#         self.desired_pose.position = Point(
#             x=self.coords[0], y=self.coords[1], z=self.coords[2]
#         )
#         quat = quaternion_from_euler(self.coords[3], self.coords[4], self.coords[5])
#         orientation = Quaternion()
#         orientation.x, orientation.y, orientation.z, orientation.w = (
#             quat[0],
#             quat[1],
#             quat[2],
#             quat[3],
#         )
#         self.desired_pose.orientation = orientation

#         # calls run
#         return super().execute(ud)

#     def run(self, ud):
#         desiredPosition = self.desired_pose.position
#         desiredOrientation = euler_from_quaternion(
#             quat_to_list(self.desired_pose.orientation)
#         )
#         print(
#             f"Moving to {tuple(pos_to_list(desiredPosition))} with orientation {desiredOrientation}."
#         )

#         # loop continues as long as conditional evaluates to false
#         while not (
#             self.state
#             and utilities.stopped_at_pose(
#                 self.state.pose.pose, self.get_desired_pose(), self.state.twist.twist
#             )
#         ):
#             self.task_state.create_rate(15)
#             rclpy.spin_once(self.task_state)

#             if self.state is not None:
#                 currPose = self.state.pose.pose
#                 currPos = currPose.position
#                 currAtt = euler_from_quaternion(quat_to_list(currPose.orientation))
#                 print(
#                     f"Current Position: {tuple(pos_to_list(currPos))}\tCurrent Attitude: {currAtt}"
#                 )
#                 print(
#                     f"Desired Position: {tuple(pos_to_list(desiredPosition))}\tDesired Attitude: {desiredOrientation}"
#                 )

#             print(f"not yet")
#             self.publish_desired_pose(self.get_desired_pose())
#             time.sleep(0.5)

#         print("i have arrived")
#         return "done"

#     def get_desired_pose(self) -> Pose:
#         return self.desired_pose


# class MoveToPoseLocalTask(MoveToPoseGlobalTask):
#     # Move to pose given in local coordinates
#     def __init__(self, x, y, z, roll, pitch, yaw):
#         super().__init__(x, y, z, roll, pitch, yaw)

#     def run(self, ud):
#         # Transform the desired pose from base_link frame to odom frame
#         # aka convert to global
#         self.desired_pose = utilities.transform("base_link", "odom", self.desired_pose)

#         return super().run(ud)


# class AllocateVelocityLocalTask(Task):
#     def __init__(self, x, y, z, roll, pitch, yaw):
#         super().__init__(task_name="allocate_local_velocity", outcomes=["done"])
#         linear = Vector3(x=x, y=y, z=z)
#         angular = Vector3(x=roll, y=pitch, z=yaw)
#         self.desired_twist = Twist(linear=linear, angular=angular)

#     def run(self, ud):
#         self.publish_desired_twist(self.desired_twist)
#         return "done"


# class AllocateVelocityGlobalTask(AllocateVelocityLocalTask):
#     # Given global velocity, we first convert into local velocity
#     def __init__(self, x, y, z, roll, pitch, yaw):
#         super().__init__(x, y, z, roll, pitch, yaw)
#         odom_global = Odometry()
#         odom_global.twist.twist = self.desired_twist
#         odom_local = utilities.transform("odom", "base_link", odom_global)
#         self.desired_twist = odom_local.twist.twist
