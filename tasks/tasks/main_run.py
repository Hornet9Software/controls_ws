import json
import subprocess
import time

import numpy as np
import rclpy
from simple_node import Node
from tasks.movement_tasks import *
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

with open("path.json", "r") as f:
    PATH = json.load(f)


class SM(Node):

    def __init__(self):
        super().__init__("task_node")

        sm = StateMachine(outcomes=["finish"])

        sm.add_state(
            "CALIBRATE",
            HoldForTime(
                outcomes=["done"],
                time_to_hold=10,
                target_depth=-1.0,
                targetRPY=[0.0, 0.0, 0.0],
            ),
            transitions={"done": "START_TO_ORANGE_FLARE"},
        )

        sm.add_state(
            "START_TO_ORANGE_FLARE",
            MoveDistance(
                outcomes=["done"],
                distance=PATH[0][0],
                target_depth=-1.0,
                targetRPY=[0.0, 0.0, PATH[0][1]],
                eqm_time=10,
            ),
            transitions={"done": "ORANGE_FLARE_TO_GATE"},
        )

        sm.add_state(
            "ORANGE_FLARE_TO_GATE",
            MoveToObject(
                outcomes=["done"],
                object_name="yellow_flare",
                target_depth=-1.0,
                distance_threshold=1.5,
                targetRPY=[0.0, 0.0, PATH[1][1]],
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "GATE_TO_ANCHOR"},
        )

        sm.add_state(
            "GATE_TO_ANCHOR",
            MoveDistance(
                outcomes=["done"],
                distance=PATH[2][0],
                target_depth=-1.0,
                targetRPY=[0.0, 0.0, PATH[2][1]],
                eqm_time=10,
            ),
            transitions={"done": "ANCHOR_TO_FIRST_FLARE"},
        )

        sm.add_state(
            "ANCHOR_TO_FIRST_FLARE",
            MoveToObject(
                outcomes=["done"],
                object_name="red_flare",
                target_depth=-1.0,
                distance_threshold=1.5,
                targetRPY=[0.0, 0.0, PATH[3][1]],
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "FIRST_FLARE_TO_ANCHOR"},
        )

        sm.add_state(
            "FIRST_FLARE_TO_ANCHOR",
            MoveDistance(
                outcomes=["done"],
                distance=PATH[4][0],
                target_depth=-1.0,
                targetRPY=[0.0, 0.0, PATH[4][1]],
                eqm_time=10,
            ),
            transitions={"done": "ANCHOR_TO_SECOND_FLARE"},
        )

        sm.add_state(
            "ANCHOR_TO_SECOND_FLARE",
            MoveToObject(
                outcomes=["done"],
                object_name="red_flare",
                target_depth=-1.0,
                distance_threshold=1.5,
                targetRPY=[0.0, 0.0, PATH[5][1]],
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "SECOND_FLARE_TO_ANCHOR"},
        )

        sm.add_state(
            "SECOND_FLARE_TO_ANCHOR",
            MoveDistance(
                outcomes=["done"],
                distance=PATH[6][0],
                target_depth=-1.0,
                targetRPY=[0.0, 0.0, PATH[6][1]],
                eqm_time=10,
            ),
            transitions={"done": "ANCHOR_TO_THIRD_FLARE"},
        )

        sm.add_state(
            "ANCHOR_TO_THIRD_FLARE",
            MoveToObject(
                outcomes=["done"],
                object_name="red_flare",
                target_depth=-1.0,
                distance_threshold=1.5,
                targetRPY=[0.0, 0.0, PATH[7][1]],
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "THIRD_FLARE_TO_BUCKETS"},
        )

        sm.add_state(
            "THIRD_FLARE_TO_BUCKETS",
            MoveDistance(
                outcomes=["done"],
                distance=PATH[8][0],
                target_depth=-1.0,
                targetRPY=[0.0, 0.0, PATH[8][1]],
                eqm_time=10,
            ),
            transitions={"done": "SURFACE"},
        )

        sm.add_state(
            "SURFACE",
            HoldForTime(
                outcomes=["done"],
                time_to_hold=10,
                target_depth=-0.2,
                targetRPY=[0.0, 0.0, 0.0],
            ),
            transitions={"done": "finish"},
        )

        outcome = sm()

        YasminViewerPub(self, "HORNET", sm)
        print(outcome)


def main(args=None):
    rclpy.init(args=args)

    try:
        Task.init_task_state()
        state_machine = SM()
        state_machine.join_spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        subprocess.run(["cansend", "can0", "000#7F7F7F7F7F7F7F7F"])
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
