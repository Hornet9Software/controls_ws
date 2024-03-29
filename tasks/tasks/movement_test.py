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

        # sm.add_state(
        #     "HOLD",
        #     HoldForTime(
        #         outcomes=["done"],
        #         time_to_hold=10,
        #         target_depth=-1.0,
        #         targetRPY=[0.0, 0.0, np.radians(20)],
        #     ),
        #     # transitions={"done": "MOVE_TO_GATE"},
        #     transitions={"done": "MOVE1"},
        # )

        # sm.add_state(
        #     "MOVE1",
        #     MoveDistance(
        #         outcomes=["done"],
        #         distance=8,
        #         target_depth=-1.0,
        #         targetRPY=[0.0, 0.0, np.radians(5)],
        #         eqm_time=10,
        #     ),
        #     transitions={"done": "MOVE2"},
        # )

        # sm.add_state(
        #     "MOVE2",
        #     MoveDistance(
        #         outcomes=["done"],
        #         distance=3,
        #         target_depth=-1.0,
        #         targetRPY=[0.0, 0.0, -np.radians(10)],
        #         eqm_time=10,
        #     ),
        #     transitions={"done": "MOVE_TO_GATE"},
        #     # transitions={"done": "finish"},
        # )

        # sm.add_state(
        #     "MOVE_TO_GATE",
        #     MoveToObject(
        #         outcomes=["done"],
        #         object_name="gate",
        #         target_depth=-1.0,
        #         distance_threshold=2.0,
        #         targetRPY=[0.0, 0.0, 0.0],
        #     ),
        #     transitions={"done": "finish"},
        # )

        sm.add_state(
            "MOVE_TO_GATE",
            MoveToObject(
                outcomes=["done"],
                object_name="red_flare",
                target_depth=-1.0,
                distance_threshold=1.5,
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
