import json
import logging
import subprocess
import time

import numpy as np
import rclpy
from simple_node import Node
from tasks.movement_tasks import *
from tasks.planner import PathPlanner
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

FLARE_DEPTH = -1.1
BUOY_DEPTH = -0.25


class SM(Node):

    def __init__(self):
        super().__init__("task_node")

        sm = StateMachine(outcomes=["finish"])

        self.instructions = PathPlanner().compute_before_flares()

        sm.add_state("INIT", InitTasks(), transitions={"done": "CALIBRATE"})

        sm.add_state(
            "CALIBRATE",
            HoldForTime(
                outcomes=["done", "restart", "end"],
                time_to_hold=10,
                target_depth=FLARE_DEPTH,
                targetRPY=[0.0, 0.0, self.instructions[0][1]],
            ),
            transitions={
                "done": "START_TO_ORANGE_FLARE",
                "restart": "CALIBRATE",
                "end": "SURFACE",
            },
        )

        sm.add_state(
            "START_TO_ORANGE_FLARE",
            MoveDistance(
                outcomes=["done", "restart", "end"],
                distance=self.instructions[0][0],
                target_depth=FLARE_DEPTH,
                targetRPY=[0.0, 0.0, self.instructions[0][1]],
                eqm_time=0,
            ),
            transitions={
                "done": "ORANGE_FLARE_TO_GATE",
                "restart": "CALIBRATE",
                "end": "SURFACE",
            },
        )

        sm.add_state(
            "ORANGE_FLARE_TO_GATE",
            MoveToGate(
                outcomes=["done", "restart", "end"],
                object_name="gate",
                target_depth=FLARE_DEPTH,
                distance_threshold=2.5,
                targetRPY=[0.0, 0.0, self.instructions[1][1]],
                completion_time_threshold=20.0,
                eqm_time=5.0,
            ),
            transitions={
                "done": "GATE_TO_GATE_FRONT",
                "restart": "CALIBRATE",
                "end": "SURFACE",
            },
        )

        sm.add_state(
            "GATE_TO_GATE_FRONT",
            MoveDistance(
                outcomes=["done", "restart", "end"],
                distance=self.instructions[2][0],
                target_depth=FLARE_DEPTH,
                targetRPY=[0.0, 0.0, self.instructions[2][1]],
                eqm_time=5,
            ),
            transitions={
                "done": "READ_COMMS_BUOY",
                "restart": "CALIBRATE",
                "end": "SURFACE",
            },
        )

        sm.add_state(
            "READ_COMMS_BUOY",
            ReadCommsBuoy(
                outcomes=["done", "restart", "end"],
                target_depth=BUOY_DEPTH,
                targetRPY=[0, 0, self.instructions[3][1]],
                wait_before_abort=15,
                eqm_time=7.0,
            ),
            transitions={
                "done": "GATE_TO_ANCHOR",
                "restart": "CALIBRATE",
                "end": "SURFACE",
            },
        )

        sm.add_state(
            "GATE_TO_ANCHOR",
            MoveDistance(
                outcomes=["done", "restart", "end"],
                distance=self.instructions[4][0],
                target_depth=FLARE_DEPTH,
                targetRPY=[0.0, 0.0, self.instructions[4][1]],
                eqm_time=7,
            ),
            transitions={
                "done": "FIRST_FLARE",
                "restart": "CALIBRATE",
                "end": "SURFACE",
            },
        )

        sm.add_state(
            "FIRST_FLARE",
            LazyHitFlare(
                outcomes=["done"],
                target_depth=FLARE_DEPTH,
                distance_threshold=2.0,
                targetRPY=[0, 0, np.radians(90)],
                completion_time_threshold=30.0,
                angle_step=0.03,
            ),
            transitions={
                "done": "SECOND_FLARE",
                "restart": "CALIBRATE",
                "end": "SURFACE",
            },
        )

        sm.add_state(
            "SECOND_FLARE",
            LazyHitFlare(
                outcomes=["done"],
                target_depth=FLARE_DEPTH,
                distance_threshold=2.0,
                targetRPY=[0, 0, np.radians(90)],
                completion_time_threshold=30.0,
                angle_step=0.03,
            ),
            transitions={
                "done": "THIRD_FLARE",
                "restart": "CALIBRATE",
                "end": "SURFACE",
            },
        )

        sm.add_state(
            "THIRD_FLARE",
            LazyHitFlare(
                outcomes=["done"],
                target_depth=FLARE_DEPTH,
                distance_threshold=2.0,
                targetRPY=[0, 0, np.radians(90)],
                completion_time_threshold=30.0,
                angle_step=0.03,
            ),
            transitions={"done": "SURFACE", "restart": "CALIBRATE", "end": "SURFACE"},
        )

        sm.add_state(
            "SURFACE",
            HoldForTime(
                outcomes=["done", "restart", "end"],
                time_to_hold=10,
                target_depth=-0.1,
                targetRPY=[0.0, 0.0, 0.0],
            ),
            transitions={"done": "SURFACE", "restart": "CALIBRATE", "end": "SURFACE"},
        )

        outcome = sm()

        YasminViewerPub(self, "HORNET", sm)
        print(outcome)


def main(args=None):
    rclpy.init(args=args)

    try:
        logging.info("STARTED MAIN RUN...")
        Task.init_task_state()
        state_machine = SM()
        state_machine.join_spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        subprocess.run(["cansend", "can0", "000#7F7F7F7F7F7F7F7F"])
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
