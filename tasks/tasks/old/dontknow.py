import json
import subprocess
import time

import numpy as np
import rclpy
from simple_node import Node
from tasks.movement_tasks import *
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub


class SM(Node):

    DEPTH = -0.35

    def __init__(self):
        super().__init__("task_node")

        sm = StateMachine(outcomes=["finish"])

        sm.add_state(
            "FIX_DEPTH",
            HoldForTime(
                outcomes=["done"],
                time_to_hold=5,
                target_depth=SM.DEPTH,
                targetRPY=[0, 0, 0],
            ),
            transitions={"done": "QUALIFICATION"},
        )

        sm.add_state(
            "QUALIFICATION",
            Qualification(
                outcomes=["correct_depth", "done"],
                distance=15,
                target_depth=SM.DEPTH,
                depth_threshold=0.2,
                targetRPY=[0, 0, 0],
                override_forward_acceleration=3,
            ),
            transitions={
                "correct_depth": "FIX_DEPTH",
                "done": "finish",
            },
        )

        outcome = sm()

        YasminViewerPub(self, "HORNET", sm)
        print(outcome)


def main(args=None):
    # delay 20 seconds
    # for i in range(10):
    #     subprocess.run(["cansend", "can0", "000#7F7F7F7F7F7F7F7F"])
    #     time.sleep(1)

    # for i in range(20):
    #     subprocess.run(["cansend", "can0", "000#7F7F7F7F7F7F7F7F"])
    #     time.sleep(0.5)

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
