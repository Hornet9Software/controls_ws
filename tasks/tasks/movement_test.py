import time

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
            "HOLD",
            HoldForTime(
                outcomes=["done"],
                time_to_hold=20,
                target_depth=-1.2,
                targetRPY=[0.0, 0.0, 0.0],
            ),
            transitions={"done": "finish"},
        )

        outcome = sm()

        YasminViewerPub(self, "HORNET", sm)
        print(outcome)


def main(args=None):
    rclpy.init(args=args)

    Task.init_task_state()

    state_machine = SM()
    state_machine.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
