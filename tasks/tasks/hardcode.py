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

FLARE_DEPTH = -1.0
BUOY_DEPTH = -0.25


class SM(Node):

    def __init__(self):
        super().__init__("task_node")

        sm = StateMachine(outcomes=["finish"])

        self.instructions = PathPlanner().compute_before_flares()

        sm.add_state(
            "CALIBRATE",
            HoldForTime(
                outcomes=["done", "restart"],
                time_to_hold=10,
                target_depth=FLARE_DEPTH,
                targetRPY=[0.0, 0.0, 0.0],
            ),
            transitions={"done": "START_TO_ORANGE_FLARE", "restart": "CALIBRATE"},
        )

        sm.add_state(
            "START_TO_ORANGE_FLARE",
            MoveDistance(
                outcomes=["done", "restart"],
                distance=self.instructions[0][0],
                target_depth=FLARE_DEPTH,
                targetRPY=[0.0, 0.0, self.instructions[0][1]],
                eqm_time=7,
            ),
            transitions={
                "done": "ORANGE_FLARE_TO_GATE",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "ORANGE_FLARE_TO_GATE",
            MoveToGate(
                outcomes=["done", "restart"],
                object_name="gate",
                target_depth=FLARE_DEPTH,
                distance_threshold=2.5,
                targetRPY=[0.0, 0.0, self.instructions[1][1]],
                completion_time_threshold=20.0,
            ),
            transitions={
                "done": "MOVE_IN_FRONT_OF_GATE",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "MOVE_IN_FRONT_OF_GATE",
            MoveDistance(
                outcomes=["done", "restart"],
                distance=self.instructions[2][0],
                target_depth=FLARE_DEPTH,
                targetRPY=[0.0, 0.0, self.instructions[2][1]],
                eqm_time=5,
            ),
            transitions={"done": "READ_COMMS_BUOY", "restart": "CALIBRATE"},
        )

        sm.add_state(
            "READ_COMMS_BUOY",
            ReadCommsBuoy(
                outcomes=["done", "restart"],
                target_depth=BUOY_DEPTH,
                targetRPY=[0, 0, self.instructions[3][1]],
                wait_before_abort=10,
                eqm_time=10.0,
            ),
            transitions={"done": "GATE_TO_ANCHOR", "restart": "CALIBRATE"},
        )

        sm.add_state(
            "GATE_TO_ANCHOR",
            MoveDistance(
                outcomes=["done", "restart"],
                distance=self.instructions[4][0],
                target_depth=FLARE_DEPTH,
                targetRPY=[0.0, 0.0, self.instructions[4][1]],
                eqm_time=10,
            ),
            transitions={
                "done": "ANCHOR_TO_FIRST_FLARE",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "ANCHOR_TO_FIRST_FLARE",
            HitFlare(
                flare_number=1,
                outcomes=["done", "restart"],
                target_depth=FLARE_DEPTH,
                distance_threshold=2,
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "CLEAR_FLARE_1", "restart": "CALIBRATE"},
        )

        sm.add_state(
            "CLEAR_FLARE_1",
            MoveDistance(
                outcomes=["done", "restart"],
                distance=1,
                target_depth=FLARE_DEPTH,
                targetRPY=[0, 0, np.radians(-90)],
                eqm_time=5,
            ),
            transitions={
                "done": "FINISH_CLEAR_FLARE_1",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "FINISH_CLEAR_FLARE_1",
            MoveDistance(
                outcomes=["done", "restart"],
                distance=1,
                target_depth=FLARE_DEPTH,
                targetRPY=[0, 0, np.radians(90)],
                eqm_time=5,
            ),
            transitions={
                "done": "FIRST_FLARE_TO_ANCHOR",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "FIRST_FLARE_TO_ANCHOR",
            MoveFromFlare(
                flare_number=1,
                outcomes=["done", "restart"],
                target_depth=FLARE_DEPTH,
                eqm_time=10,
            ),
            transitions={
                "done": "ANCHOR_TO_SECOND_FLARE",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "ANCHOR_TO_SECOND_FLARE",
            HitFlare(
                flare_number=2,
                outcomes=["done", "restart"],
                target_depth=FLARE_DEPTH,
                distance_threshold=2,
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "CLEAR_FLARE_2", "restart": "CALIBRATE"},
        )

        sm.add_state(
            "CLEAR_FLARE_2",
            MoveDistance(
                outcomes=["done", "restart"],
                distance=1,
                target_depth=FLARE_DEPTH,
                targetRPY=[0, 0, np.radians(-90)],
                eqm_time=5,
            ),
            transitions={
                "done": "FINISH_CLEAR_FLARE_2",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "FINISH_CLEAR_FLARE_2",
            MoveDistance(
                outcomes=["done", "restart"],
                distance=1,
                target_depth=FLARE_DEPTH,
                targetRPY=[0, 0, np.radians(90)],
                eqm_time=5,
            ),
            transitions={
                "done": "SECOND_FLARE_TO_ANCHOR",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "SECOND_FLARE_TO_ANCHOR",
            MoveFromFlare(
                flare_number=2,
                outcomes=["done", "restart"],
                target_depth=FLARE_DEPTH,
                eqm_time=10,
            ),
            transitions={
                "done": "ANCHOR_TO_THIRD_FLARE",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "ANCHOR_TO_THIRD_FLARE",
            HitFlare(
                flare_number=3,
                outcomes=["done", "restart"],
                target_depth=FLARE_DEPTH,
                distance_threshold=2,
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "CLEAR_FLARE_3", "restart": "CALIBRATE"},
        )

        sm.add_state(
            "CLEAR_FLARE_3",
            MoveDistance(
                outcomes=["done", "restart"],
                distance=1,
                target_depth=FLARE_DEPTH,
                targetRPY=[0, 0, np.radians(-90)],
                eqm_time=5,
            ),
            transitions={
                "done": "FINISH_CLEAR_FLARE_3",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "FINISH_CLEAR_FLARE_3",
            MoveDistance(
                outcomes=["done", "restart"],
                distance=1,
                target_depth=FLARE_DEPTH,
                targetRPY=[0, 0, np.radians(90)],
                eqm_time=5,
            ),
            transitions={
                "done": "THIRD_FLARE_TO_BUCKETS",
                "restart": "CALIBRATE",
            },
        )

        sm.add_state(
            "THIRD_FLARE_TO_BUCKETS",
            MoveFromFlare(
                flare_number=3,
                outcomes=["done", "restart"],
                target_depth=FLARE_DEPTH,
                eqm_time=10,
            ),
            transitions={"done": "SURFACE", "restart": "CALIBRATE"},
        )

        sm.add_state(
            "SURFACE",
            HoldForTime(
                outcomes=["done", "restart"],
                time_to_hold=10,
                target_depth=-0.2,
                targetRPY=[0.0, 0.0, 0.0],
            ),
            transitions={"done": "SURFACE", "restart": "CALIBRATE"},
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
