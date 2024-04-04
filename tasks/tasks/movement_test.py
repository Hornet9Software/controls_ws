import subprocess
import time

import numpy as np
import rclpy
from simple_node import Node
from tasks.movement_tasks import *
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

flare_depth = -1.2


class SM(Node):
    def __init__(self):
        super().__init__("task_node")

        sm = StateMachine(outcomes=["finish"])

        sm.add_state(
            "HOLD",
            HoldForTime(
                outcomes=["done"],
                time_to_hold=5,
                target_depth=flare_depth,
                targetRPY=[0.0, 0.0, np.radians(180)],
            ),
            # transitions={"done": "MOVE_TO_GATE"},
            transitions={"done": "4_TO_5"},
        )

        sm.add_state(
            "4_TO_5",
            MoveToObject(
                outcomes=["done"],
                object_name="red_flare",
                target_depth=flare_depth,
                distance_threshold=1.5,
                targetRPY=[0.0, 0.0, np.radians(180)],
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "5_TO_4"},
        )

        sm.add_state(
            "5_TO_4",
            MoveDistance(
                outcomes=["done"],
                distance=3,
                target_depth=flare_depth,
                targetRPY=[0.0, 0.0, np.radians(0)],
                eqm_time=10,
            ),
            transitions={"done": "4_TO_6"},
        )

        sm.add_state(
            "4_TO_6",
            MoveToObject(
                outcomes=["done"],
                object_name="yellow_flare",
                target_depth=flare_depth,
                distance_threshold=1.5,
                targetRPY=[0.0, 0.0, np.radians(180)],
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "6_TO_4"},
        )

        sm.add_state(
            "6_TO_4",
            MoveDistance(
                outcomes=["done"],
                distance=3,
                target_depth=flare_depth,
                targetRPY=[0.0, 0.0, np.radians(0)],
                eqm_time=10,
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
