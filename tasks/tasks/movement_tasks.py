import logging
import time
from copy import copy, deepcopy

import numpy as np
from controls_core.params import angle_abs_error
from numpy import radians
from tasks.planner import PathPlanner
from tasks.task import Task


class InitTasks(Task):
    def __init__(self, outcomes):
        super().__init__(outcomes)

    def run(self, blackboard):
        blackboard.order = PathPlanner().default_flare_order
        blackboard.instructions = PathPlanner().compute_flares()
        blackboard.attempted_flares = []

        return "done"


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

        self.logger.info("")
        self.logger.info("HOLDING FOR TIME...")
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
        override_forward_acceleration=3.0,
    ):
        super().__init__(outcomes)

        self.distance = distance
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY
        self.eqm_time = eqm_time
        self.override_forward_acceleration = override_forward_acceleration

    @staticmethod
    def custom_acceleration(curr_time, forward_time, expected_acceleration):
        end_of_rise_acceleration = 2
        gradient = expected_acceleration / end_of_rise_acceleration

        if (forward_time < end_of_rise_acceleration) or (
            curr_time > end_of_rise_acceleration
        ):
            return expected_acceleration
        else:
            return gradient * curr_time

    def run(self, blackboard):
        self.logger.info("")
        self.logger.info(f"MOVING {self.distance}M ...")

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
                override_forward_acceleration=self.override_forward_acceleration,
                # override_forward_acceleration=MoveDistance.custom_acceleration(
                #     delta_t - self.eqm_time,
                #     forward_time,
                #     self.override_forward_acceleration,
                # ),
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


class Qualification(Task):
    def __init__(
        self,
        outcomes=["done"],
        distance=13,
        target_depth=-0.6,
        depth_threshold=0.2,
        targetRPY=[0, 0, 0],
        override_forward_acceleration=3.0,
    ):
        super().__init__(outcomes)

        self.distance = distance
        self.depth_threshold = depth_threshold
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY
        self.override_forward_acceleration = override_forward_acceleration

    @staticmethod
    def custom_acceleration(curr_time, forward_time, expected_acceleration):
        end_of_rise_acceleration = 2
        gradient = expected_acceleration / end_of_rise_acceleration

        if (forward_time < end_of_rise_acceleration) or (
            curr_time > end_of_rise_acceleration
        ):
            return expected_acceleration
        else:
            return gradient * curr_time

    def run(self, blackboard):
        self.logger.info("")
        self.logger.info(f"MOVING {self.distance}M ...")

        delta_t = self.curr_time - self.init_time

        forward_time = 1.675 * self.distance - 1.0722

        if (self.depth <= self.targetXYZ[2] - self.depth_threshold) or (
            self.depth >= self.targetXYZ[2] + self.depth_threshold
        ):
            return "correct_depth"

        if delta_t < forward_time:
            self.logger.info("ACTUALLY MOVING")
            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                override_forward_acceleration=Qualification.custom_acceleration(
                    delta_t,
                    forward_time,
                    self.override_forward_acceleration,
                ),
            )
        elif delta_t >= forward_time:
            self.logger.info("FINISHED MOVING, JUST HOLD")
            self.correctVehicle(
                self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ
            )

        time.sleep(0.1)

        return "running"


class MoveFromFlare(MoveDistance):
    def __init__(
        self,
        flare_number,
        outcomes=["done"],
        distance=5,
        target_depth=-1.2,
        targetRPY=[0, 0, 0],
        eqm_time=30,
        override_forward_acceleration=3.0,
    ):
        super().__init__(
            outcomes,
            distance,
            target_depth,
            targetRPY,
            eqm_time,
            override_forward_acceleration,
        )

        self.instruction_number = {
            1: 1,
            2: 3,
            3: 5,
        }[flare_number]

        self.first_run = True

    def run(self, blackboard):
        if self.first_run:
            self.order = blackboard.order
            self.instructions = blackboard.instructions
            self.targetRPY = [0, 0, self.instructions[self.instruction_number][1]]
            self.distance = self.instructions[self.instruction_number][0]
            self.first_run = False

        return super().run(blackboard)


class MoveToGate(Task):
    def __init__(
        self,
        outcomes=["done"],
        object_name="gate",
        target_depth=-1.2,
        distance_threshold=2,
        targetRPY=[0, 0, 0],
        completion_time_threshold=10.0,
        eqm_time=5.0,
    ):
        super().__init__(outcomes)

        # Load parameters
        self.object_name = object_name
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY
        self.distance_threshold = distance_threshold
        self.completion_time_threshold = completion_time_threshold
        self.eqm_time = eqm_time

        # Init variables
        self.last_detected_time = None

    def run(self, blackboard):

        self.clear_old_cv_data(self.object_name, refresh_time=1.0)
        delta_t = self.curr_time - self.init_time
        self.logger.info("")
        self.logger.info(f"CURRENT TIME: {self.curr_time}")
        self.logger.info(f"LAST DETECTED TIME: {self.last_detected_time}")

        # If vehicle needs time to equilibriate at the start,
        if delta_t < self.eqm_time:
            self.logger.info("EQUILIBRIATING...")
            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                pid_type="gate",
            )

        elif self.cv_data[self.object_name] is not None:
            self.last_detected_time = self.cv_data[self.object_name]["time"]

            self.targetRPY[2] = self.cv_data[self.object_name]["bearing"]
            currRPY = copy(self.currRPY)
            currRPY[2] = 0.0

            if (
                angle_abs_error(self.targetRPY[2], currRPY[2]) < 0.1
                or self.cv_data[self.object_name]["distance"] < self.distance_threshold
            ):

                self.correctVehicle(
                    currRPY,
                    self.targetRPY,
                    self.currXYZ,
                    self.targetXYZ,
                    override_forward_acceleration=1.5,
                    pid_type="gate",
                )

            # Otherwise, correct the angle but don't move the robot.
            else:
                self.correctVehicle(
                    currRPY,
                    self.targetRPY,
                    self.currXYZ,
                    self.targetXYZ,
                    pid_type="gate",
                )

        else:
            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                pid_type="gate",
            )

            # If the flares are not seen after a long time,
            # they have been knocked down.
            if self.last_detected_time is not None:
                self.logger.info("CHECKING IF KNOCKED DOWN...")
                if (
                    self.curr_time - self.last_detected_time
                ) >= self.completion_time_threshold:
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
        sweeping_angle=np.radians(360),
        eqm_time=10.0,
    ):
        super().__init__(outcomes)

        # Load parameters
        self.object_name = object_name
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY
        self.distance_threshold = distance_threshold
        self.completion_time_threshold = completion_time_threshold
        self.angle_step = angle_step
        self.sweeping_angle = sweeping_angle
        self.eqm_time = eqm_time

        # Init variables
        self.last_detected_time = None
        self.centre_yaw = self.targetRPY[2]
        self.total_angle = 0.0
        self.first_detection = True

    @staticmethod
    def custom_sweep(angle):
        if angle >= 0 and angle < np.pi / 2:
            return 0
        elif angle >= np.pi / 2 and angle < (2.5 * np.pi):
            return np.sin(angle - np.pi / 2)
        else:
            if angle < 0:
                return MoveToObject.custom_sweep(angle + 2.5 * np.pi)
            else:
                return MoveToObject.custom_sweep(angle - 2.5 * np.pi)

    def run(self, blackboard):

        self.clear_old_cv_data(self.object_name, refresh_time=1.0)
        delta_t = self.curr_time - self.init_time
        self.logger.info("")
        self.logger.info(f"CURRENT TIME: {self.curr_time}")
        self.logger.info(f"LAST DETECTED TIME: {self.last_detected_time}")

        # If vehicle needs time to equilibriate at the start,
        if delta_t < self.eqm_time:
            self.logger.info("EQUILIBRIATING...")
            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                pid_type="flare",
            )

        # Else if object is not detected,
        elif self.cv_data[self.object_name] is None:
            self.logger.info("NOT DETECTED!")

            if self.last_detected_time is not None and self.targetRPY[2] > 0:
                sweep_sign = 1
            else:
                sweep_sign = -1

            self.targetRPY[2] = self.centre_yaw + sweep_sign * (
                self.sweeping_angle * MoveToObject.custom_sweep(self.total_angle)
            )
            self.total_angle += self.angle_step

            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                pid_type="flare",
            )

            # If the flares are not seen after a long time,
            # they have been knocked down.
            if self.last_detected_time is not None:
                self.logger.info("CHECKING IF KNOCKED DOWN...")
                if (
                    self.curr_time - self.last_detected_time
                ) >= self.completion_time_threshold:
                    return "done"

        # If the object is detected,
        else:
            self.logger.info("DETECTED!")

            # Update target yaw and last detected time
            self.targetRPY[2] = self.cv_data[self.object_name]["bearing"]
            self.last_detected_time = self.cv_data[self.object_name]["time"]

            # Reset sweeping parameters
            self.centre_yaw = self.currRPY[2] + self.targetRPY[2]
            self.total_angle = 0.0

            # Zero curr yaw so that yaw error would be required rotation
            # to face object.
            currRPY = copy(self.currRPY)
            currRPY[2] = 0.0

            # print(self.cv_data[self.object_name]["distance"])

            # If the error in yaw is small or if close to object,
            # the robot can move.
            if (
                angle_abs_error(self.targetRPY[2], currRPY[2]) < 0.4
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
                        override_forward_acceleration=1.5,
                        pid_type="flare",
                    )

                # If not close to object and error in yaw is small, correct angle
                # and move at the same time.
                else:
                    self.correctVehicle(
                        currRPY,
                        self.targetRPY,
                        self.currXYZ,
                        self.targetXYZ,
                        override_forward_acceleration=1.5,
                        pid_type="flare",
                    )

            # Otherwise, correct the angle but don't move the robot.
            else:
                self.correctVehicle(
                    currRPY,
                    self.targetRPY,
                    self.currXYZ,
                    self.targetXYZ,
                    pid_type="flare",
                )

        time.sleep(0.1)

        return "running"


class HitFlare(MoveToObject):
    def __init__(
        self,
        flare_number,
        outcomes=["done"],
        object_name="yellow_flare",
        target_depth=-1.2,
        distance_threshold=1,
        targetRPY=[0, 0, 0],
        completion_time_threshold=30,
        angle_step=0.03,
        sweeping_angle=np.radians(360),
        eqm_time=5,
    ):
        super().__init__(
            outcomes,
            object_name,
            target_depth,
            distance_threshold,
            targetRPY,
            completion_time_threshold,
            angle_step,
            sweeping_angle,
            eqm_time,
        )

        self.instruction_number = {
            1: 0,
            2: 1,
            3: 2,
        }[flare_number]

        self.flare_number = {1: "first", 2: "second", 3: "third"}[flare_number]

        self.first_run = True

    def run(self, blackboard):
        if self.first_run:
            self.order = blackboard.order
            self.instructions = blackboard.instructions
            self.targetRPY = [0, 0, self.instructions[self.instruction_number][1]]
            self.centre_yaw = self.targetRPY[2]
            self.object_name = self.order[self.flare_number]
            self.first_run = False

        return super().run(blackboard)


class LazyHitFlare(Task):
    def __init__(
        self,
        outcomes=["done"],
        target_depth=-1.2,
        distance_threshold=2,
        targetRPY=[0, 0, 0],
        completion_time_threshold=30.0,
        angle_step=0.03,
        sweeping_angle=np.radians(360),
        eqm_time=5.0,
    ):
        super().__init__(outcomes)

        # Load parameters
        self.object_name = None
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY
        self.targetRPY[2] = self.currRPY[2]
        self.distance_threshold = distance_threshold
        self.completion_time_threshold = completion_time_threshold
        self.angle_step = angle_step
        self.sweeping_angle = sweeping_angle
        self.eqm_time = eqm_time

        # Init variables
        self.last_detected_time = None
        self.centre_yaw = self.targetRPY[2]
        self.total_angle = 0.0
        self.first_run = True

    @staticmethod
    def custom_sweep(angle):
        if angle >= 0 and angle < np.pi / 2:
            return 0
        elif angle >= np.pi / 2 and angle < (2.5 * np.pi):
            return np.sin(angle - np.pi / 2)
        else:
            if angle < 0:
                return MoveToObject.custom_sweep(angle + 2.5 * np.pi)
            else:
                return MoveToObject.custom_sweep(angle - 2.5 * np.pi)

    def run(self, blackboard):
        self.clear_old_cv_data("red_flare", refresh_time=1.0)
        self.clear_old_cv_data("yellow_flare", refresh_time=1.0)
        self.clear_old_cv_data("blue_flare", refresh_time=1.0)

        delta_t = self.curr_time - self.init_time
        self.logger.info("")
        self.logger.info(f"CURRENT TIME: {self.curr_time}")
        self.logger.info(f"LAST DETECTED TIME: {self.last_detected_time}")

        # If vehicle needs time to equilibriate at the start,
        if delta_t < self.eqm_time:
            self.logger.info("EQUILIBRIATING...")
            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                pid_type="flare",
            )

        elif self.last_detected_time is None:
            # wait for detection
            if (
                self.cv_data["yellow_flare"] is not None
                and "yellow_flare" not in blackboard.attempted_flares
            ):
                self.object_name = "yellow_flare"
                self.last_detected_time = time.time()
                blackboard.attempted_flares.append("yellow_flare")

            elif (
                self.cv_data["red_flare"] is not None
                and "red_flare" not in blackboard.attempted_flares
            ):
                self.object_name = "red_flare"
                self.last_detected_time = time.time()
                blackboard.attempted_flares.append("red_flare")

            elif (
                self.cv_data["blue_flare"] is not None
                and "blue_flare" not in blackboard.attempted_flares
            ):
                self.object_name = "blue_flare"
                self.last_detected_time = time.time()
                blackboard.attempted_flares.append("blue_flare")

            else:
                self.logger.info("NO LOCK ON!")

                self.targetRPY[2] = self.centre_yaw + (
                    self.sweeping_angle * MoveToObject.custom_sweep(self.total_angle)
                )
                self.total_angle += self.angle_step

                self.correctVehicle(
                    self.currRPY,
                    self.targetRPY,
                    self.currXYZ,
                    self.targetXYZ,
                    pid_type="flare",
                )

        # Else if object is not detected, but locked on
        elif self.cv_data[self.object_name] is None:
            self.logger.info("NOT DETECTED!")

            if self.targetRPY[2] > 0:
                sweep_sign = 1
            else:
                sweep_sign = -1

            self.targetRPY[2] = self.centre_yaw + sweep_sign * (
                self.sweeping_angle * MoveToObject.custom_sweep(self.total_angle)
            )
            self.total_angle += self.angle_step

            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                pid_type="flare",
            )

            # If the flares are not seen after a long time,
            # they have been knocked down.
            self.logger.info("CHECKING IF KNOCKED DOWN...")
            if (
                self.curr_time - self.last_detected_time
            ) >= self.completion_time_threshold:
                return "done"

        # If the object is detected,
        else:
            self.logger.info("DETECTED!")

            # Update target yaw and last detected time
            self.targetRPY[2] = self.cv_data[self.object_name]["bearing"]
            self.last_detected_time = self.cv_data[self.object_name]["time"]

            # Reset sweeping parameters
            self.centre_yaw = self.currRPY[2] + self.targetRPY[2]
            self.total_angle = 0.0

            # Zero curr yaw so that yaw error would be required rotation
            # to face object.
            currRPY = copy(self.currRPY)
            currRPY[2] = 0.0

            # If the error in yaw is small or if close to object,
            # the robot can move.
            if (
                angle_abs_error(self.targetRPY[2], currRPY[2]) < 0.4
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
                        override_forward_acceleration=1.5,
                        pid_type="flare",
                    )

                # If not close to object and error in yaw is small, correct angle
                # and move at the same time.
                else:
                    self.correctVehicle(
                        currRPY,
                        self.targetRPY,
                        self.currXYZ,
                        self.targetXYZ,
                        override_forward_acceleration=1.5,
                        pid_type="flare",
                    )

            # Otherwise, correct the angle but don't move the robot.
            else:
                self.correctVehicle(
                    currRPY,
                    self.targetRPY,
                    self.currXYZ,
                    self.targetXYZ,
                    pid_type="flare",
                )

        time.sleep(0.1)

        return "running"


class ReadCommsBuoy(Task):
    def __init__(
        self,
        outcomes=["done"],
        target_depth=-0.5,
        targetRPY=[0, 0, 0],
        wait_before_abort=15.0,
        eqm_time=7.0,
    ):
        super().__init__(outcomes)

        # Load parameters
        self.targetXYZ = [0.0, 0.0, target_depth]
        self.targetRPY = targetRPY
        self.wait_before_abort = wait_before_abort
        self.eqm_time = eqm_time

        self.angle_step = 0.05
        self.sweeping_angle = np.radians(45)
        self.centre_yaw = self.targetRPY[2]
        self.total_angle = 0.0

    def run(self, blackboard):

        delta_t = self.curr_time - self.init_time

        # If vehicle needs time to equilibriate at the start,
        if delta_t < self.eqm_time:
            self.logger.info("EQUILIBRIATING...")
            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                pid_type="normal",
            )

        # else if led not detected yet...
        elif self.flare_order is None:
            self.logger.info("LED NOT DETECTED!")

            self.targetRPY[2] = self.centre_yaw + (
                self.sweeping_angle * np.sin(self.total_angle)
            )
            self.total_angle += self.angle_step

            self.correctVehicle(
                self.currRPY,
                self.targetRPY,
                self.currXYZ,
                self.targetXYZ,
                pid_type="normal",
            )

            # If no detection after a long time, just pick one order and move on
            if delta_t > self.wait_before_abort:
                blackboard.order = PathPlanner().default_flare_order
                blackboard.instructions = PathPlanner().compute_flares()
                return "done"

        # If the object is detected,
        else:
            self.logger.info("LED DETECTED!")
            self.logger.info(f"FLARE ORDER: {self.flare_order}")
            blackboard.order = self.flare_order
            blackboard.instructions = PathPlanner().compute_flares(self.flare_order)
            return "done"

        time.sleep(0.1)

        return "running"
