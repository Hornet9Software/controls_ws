import time

import numpy as np
import rclpy

from simple_node import Node
from tasks.movement_tasks import *
from yasmin import StateMachine

# from yasmin.yasmin_viewer import YasminViewerPub


class SM(Node):
    def __init__(self):
        super().__init__("task_node")

        sm = StateMachine(outcomes=["finish"])

        # sm.add_state(
        #     "HOLD",
        #     HoldForTime(
        #         outcomes=["done"],
        #         time_to_hold=60,
        #         target_depth=-1.2,
        #         targetRPY=[0.0, 0.0, 0.0],
        #     ),
        #     transitions={"done": "finish"},
        # )

        # sm.add_state(
        #     "MOVE_TO_GATE",
        #     MoveToObject(
        #         outcomes=["done"],
        #         object_name="red_flare",
        #         target_depth=-1.0,
        #         targetRPY=[0.0, 0.0, 0.0],
        #     ),
        #     transitions={"done": "finish"},
        # )

        sm.add_state(
            "MOVE1",
            MoveDistance(
                outcomes=["done"],
                distance=7,
                target_depth=-1.0,
                targetRPY=[0.0, 0.0, 1.3 + np.radians(50)],
                eqm_time=10
            ),
            transitions={"done": "MOVE2"},
        )

        sm.add_state(
            "MOVE2",
            MoveDistance(
                outcomes=["done"],
                distance=2,
                target_depth=-1.0,
                targetRPY=[0.0, 0.0, 1.3],
                eqm_time=10
            ),
            transitions={"done": "MOVE_TO_GATE"},
        )

        sm.add_state(
            "MOVE_TO_GATE",
            MoveToObject(
                outcomes=["done"],
                object_name="gate",
                target_depth=-1.0,
                targetRPY=[0.0, 0.0, 1.3],
            ),
            transitions={"done": "finish"},
        )

        outcome = sm()

        # YasminViewerPub(self, "HORNET", sm)
        print(outcome)


def main(args=None):
    rclpy.init(args=args)

    Task.init_task_state()

    state_machine = SM()
    state_machine.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
