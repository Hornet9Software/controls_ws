import time
from copy import copy, deepcopy

import numpy as np
from controls_core.params import angle_abs_error
from tasks.task import Task


class HoldForTime(Task):
    def __init__(
        self,
        outcomes=["done"],
        time_to_hold=20,
        target_depth=-1.2,
        targetRPY=[0, 0, 0],
    ):
        super().__init__(outcomes)

        self.time_to_hold = time_to_hold
        self.init_time = time.time()
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY

    def run(self, blackboard):
        self.curr_time = time.time()
        if self.curr_time - self.init_time > self.time_to_hold:
            return "done"

        self.correctVehicle(self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ)

        time.sleep(0.1)

        return "running"


class MoveDistance(Task):
    def __init__(
        self,
        outcomes=["done"],
        distance=5,
        target_depth=-1.2,
        targetRPY=[0, 0, 0],
        eqm_time=30,
    ):
        super().__init__(outcomes)

        self.distance = distance
        self.first = True
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY
        self.eqm_time = eqm_time

    def run(self, blackboard):
        if self.first:
            self.init_time = time.time()
            self.first = False

        self.curr_time = time.time()
        delta_t = self.curr_time - self.init_time

        forward_time = 1.675 * self.distance - 1.0722

        if delta_t < self.eqm_time:
            self.correctVehicle(
                self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ
            )
        elif delta_t >= self.eqm_time and delta_t < self.eqm_time + forward_time:
            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                override_forward_acceleration=3.0,
            )
        elif (
            delta_t >= self.eqm_time + forward_time
            and delta_t <= 2 * self.eqm_time + forward_time
        ):
            self.correctVehicle(
                self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ
            )
        else:
            return "done"

        time.sleep(0.1)

        return "running"


class MoveToObject(Task):
    def __init__(
        self,
        outcomes=["done"],
        object_name="gate",
        target_depth=-1.2,
        distance_threshold=2,
        targetRPY=[0, 0, 0],
        completion_time_threshold=3.0,
    ):
        super().__init__(outcomes)

        self.object_name = object_name

        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY

        self.distance_threshold = distance_threshold

        self.completion_time_threshold = completion_time_threshold

        self.centre_yaw = self.targetRPY[2]

        self.angle_step = 0.01
        self.total_angle = 0.0
        self.sweeping_angle = np.radians(45)

    def run(self, blackboard):

        self.clear_old_cv_data(self.object_name, refresh_time=1.0)

        if self.cv_data[self.object_name] is None:
            curr_time = time.time()
            if (curr_time - self.last_detected_time) >= self.completion_time_threshold:
                return "done"

            print("NOT DETECTED")
            self.targetRPY[2] = self.centre_yaw + (
                self.sweeping_angle * np.sin(self.total_angle)
            )
            self.total_angle += self.angle_step
            self.correctVehicle(
                self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ
            )

            time.sleep(0.1)

            return "running"

        print("DETECTED!")
        self.targetRPY[2] = self.cv_data[self.object_name]["bearing"]
        self.last_detetcted_time = self.cv_data[self.object_name]["time"]

        # Reset sweeping parameters
        self.centre_yaw = self.targetRPY[2]
        self.total_angle = 0.0

        currRPY = copy(self.currRPY)
        currRPY[2] = 0.0

        if (
            angle_abs_error(self.targetRPY[2], currRPY[2]) < 0.1
            or self.cv_data[self.object_name]["distance"] < self.distance_threshold
        ):
            if self.cv_data[self.object_name]["distance"] < self.distance_threshold:
                # set yaw angular acceleration to 0
                targetRPY = [0, 0, 0]
                self.correctVehicle(
                    currRPY,
                    targetRPY,
                    self.currXYZ,
                    self.targetXYZ,
                    override_forward_acceleration=2.0,
                )
            else:
                self.correctVehicle(
                    currRPY,
                    self.targetRPY,
                    self.currXYZ,
                    self.targetXYZ,
                    override_forward_acceleration=2.0,
                )
        else:
            self.correctVehicle(
                currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
            )

        time.sleep(0.1)

        return "running"
