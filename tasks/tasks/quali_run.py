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

    def __init__(self):
        super().__init__("task_node")

        sm = StateMachine(outcomes=["finish"])

        sm.add_state(
            "MOVE_THROUGH_QUALI_GATE",
            MoveDistance(
                outcomes=["done"],
                distance=11.0,
                target_depth=-0.9,
                targetRPY=[0.0, 0.0, np.radians(-10)],
                eqm_time=10,
                override_forward_acceleration=3.0,
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
