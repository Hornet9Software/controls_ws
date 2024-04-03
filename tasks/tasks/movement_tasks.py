import time
from copy import copy, deepcopy

import numpy as np
from controls_core.params import angle_abs_error
from planner import PathPlanner
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
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY

    def run(self, blackboard):

        delta_t = self.curr_time - self.init_time

        if delta_t > self.time_to_hold:
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
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY
        self.eqm_time = eqm_time

    def run(self, blackboard):

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
        completion_time_threshold=10.0,
        angle_step=0.02,
        # start_sweep_delay=1.0,
        sweeping_angle=np.radians(80),
        eqm_time=5.0,
    ):
        super().__init__(outcomes)

        # Load parameters
        self.object_name = object_name
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY
        self.distance_threshold = distance_threshold
        self.completion_time_threshold = completion_time_threshold
        self.angle_step = angle_step
        # self.start_sweep_delay = start_sweep_delay
        self.sweeping_angle = sweeping_angle
        self.eqm_time = eqm_time

        # Init variables
        self.last_detected_time = None
        self.centre_yaw = self.targetRPY[2]
        self.total_angle = 0.0
        self.first_detection = True

    def run(self, blackboard):

        self.clear_old_cv_data(self.object_name, refresh_time=1.0)
        delta_t = self.curr_time - self.init_time
        print()
        print(f"CURRENT TIME: {self.curr_time}")
        print(f"LAST DETECTED TIME: {self.last_detected_time}")

        # If vehicle needs time to equilibriate at the start,
        if delta_t < self.eqm_time:
            print("EQUILIBRIATING...")
            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                use_camera_pid=True,
            )

        # Else if object is not detected,
        elif self.cv_data[self.object_name] is None:
            print("NOT DETECTED!")

            # If time since last detection is less than
            # delay to sweep, search in the direction of
            # the last detection.
            # if (
            #     self.last_detected_time is not None
            #     and curr_time - self.last_detected_time < self.start_sweep_delay
            # ):
            #     self.correctVehicle(
            #         currRPY,
            #         self.targetRPY,
            #         self.currXYZ,
            #         self.targetXYZ,
            #         override_forward_acceleration=2.0,
            #         use_camera_pid=True,
            #     )

            if self.last_detected_time is not None and self.targetRPY[2] < 0:
                sweep_sign = -1
            else:
                sweep_sign = 1

            self.targetRPY[2] = self.centre_yaw + sweep_sign * (
                self.sweeping_angle * np.sin(self.total_angle)
            )
            self.total_angle += self.angle_step

            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                use_camera_pid=True,
            )

            # If the flares are not seen after a long time,
            # they have been knocked down.
            if self.last_detected_time is not None:
                print("CHECKING IF KNOCKED DOWN...")
                if (
                    self.curr_time - self.last_detected_time
                ) >= self.completion_time_threshold:
                    return "done"

        # If the object is detected,
        else:
            print("DETECTED!")

            # Update target yaw and last detected time
            self.targetRPY[2] = self.cv_data[self.object_name]["bearing"]
            self.last_detected_time = self.cv_data[self.object_name]["time"]

            # Reset sweeping parameters
            self.centre_yaw = self.currRPY[2]
            self.total_angle = 0.0

            # Zero curr yaw so that yaw error would be required rotation
            # to face object.
            currRPY = copy(self.currRPY)
            currRPY[2] = 0.0

            # If the error in yaw is small or if close to object,
            # the robot can move.
            if (
                angle_abs_error(self.targetRPY[2], currRPY[2]) < 0.1
                or self.cv_data[self.object_name]["distance"] < self.distance_threshold
            ):
                # If close to object, just move straight.
                if self.cv_data[self.object_name]["distance"] < self.distance_threshold:
                    # set yaw angular acceleration to 0
                    targetRPY = [0, 0, 0]
                    self.correctVehicle(
                        currRPY,
                        targetRPY,
                        self.currXYZ,
                        self.targetXYZ,
                        override_forward_acceleration=2.0,
                        use_camera_pid=True,
                    )

                # If not close to object and error in yaw is small, correct angle
                # and move at the same time.
                else:
                    self.correctVehicle(
                        currRPY,
                        self.targetRPY,
                        self.currXYZ,
                        self.targetXYZ,
                        override_forward_acceleration=2.0,
                        use_camera_pid=True,
                    )

            # Otherwise, correct the angle but don't move the robot.
            else:
                self.correctVehicle(
                    currRPY,
                    self.targetRPY,
                    self.currXYZ,
                    self.targetXYZ,
                    use_camera_pid=True,
                )

        time.sleep(0.1)

        return "running"


class ReadCommsBuoy(Task):
    def __init__(
        self,
        outcomes=["done"],
        target_depth=-0.5,
        targetRPY=[0, 0, 0],
        wait_before_abort=50.0,
        eqm_time=5.0,
    ):
        super().__init__(outcomes)

        # Load parameters
        self.order = {"first_flare": None, "second_flare": None, "third_flare": None}
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY
        self.wait_before_abort = wait_before_abort
        self.eqm_time = eqm_time

        self.angle_step = 0.05
        self.sweeping_angle = np.radians(30)
        self.centre_yaw = self.targetRPY[2]
        self.total_angle = 0.0

    def run(self, blackboard):

        delta_t = self.curr_time - self.init_time

        # If vehicle needs time to equilibriate at the start,
        if delta_t < self.eqm_time:
            print("EQUILIBRIATING...")
            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                use_camera_pid=True,
            )

        # else if led not detected yet...
        elif self.flare_order is None:
            print("NOT DETECTED!")

            self.targetRPY[2] = self.centre_yaw + (
                self.sweeping_angle * np.sin(self.total_angle)
            )
            self.total_angle += self.angle_step

            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                use_camera_pid=True,
            )

            # If no detection after a long time, just pick one order and move on
            if delta_t > self.wait_before_abort:
                blackboard.order = PathPlanner().compute_flares(
                    ["red_flare", "yellow_flare", "blue_flare"]
                )
                return "done"

        # If the object is detected,
        else:
            print("DETECTED!")
            blackboard.order = PathPlanner().compute_flares(self.flare_order)
            return "done"

        time.sleep(0.1)

        return "running"
