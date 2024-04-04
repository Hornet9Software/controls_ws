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

        self.instructions = PathPlanner().compute_before_flares()

        sm.add_state(
            "HOLD",
            HoldForTime(
                outcomes=["done"],
                time_to_hold=8,
                target_depth=flare_depth,
                targetRPY=[0.0, 0.0, np.radians(0)],
            ),
            # transitions={"done": "MOVE_TO_GATE"},
            # transitions={"done": "4_TO_5"},
            transitions={"done": "ORANGE_FLARE_TO_GATE"},
        )

        sm.add_state(
            "ORANGE_FLARE_TO_GATE",
            MoveToGate(
                outcomes=["done"],
                object_name="gate",
                target_depth=-0.8,
                distance_threshold=2.5,
                targetRPY=[0.0, 0.0, 0.0],
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "READ_COMMS_BUOY"},
        )

        # sm.add_state(
        #     "HOLD",
        #     HoldForTime(
        #         outcomes=["done"],
        #         time_to_hold=8,
        #         target_depth=flare_depth,
        #         targetRPY=[0.0, 0.0, np.radians(0)],
        #     ),
        #     # transitions={"done": "MOVE_TO_GATE"},
        #     # transitions={"done": "4_TO_5"},
        #     transitions={"done": "READ_COMMS_BUOY"},
        # )

        # sm.add_state(
        #     "READ_COMMS_BUOY",
        #     ReadCommsBuoy(
        #         outcomes=["done"],
        #         target_depth=-0.8,
        #         targetRPY=[0, 0, self.instructions[2][1]],
        #         wait_before_abort=10,
        #         eqm_time=5.0,
        #     ),
        #     transitions={"done": "GATE_TO_ANCHOR"},
        # )

        sm.add_state(
            "GATE_TO_ANCHOR",
            MoveDistance(
                outcomes=["done"],
                distance=self.instructions[3][0],
                target_depth=-1.0,
                targetRPY=[0.0, 0.0, self.instructions[3][1]],
                eqm_time=10,
            ),
            transitions={"done": "ANCHOR_TO_FIRST_FLARE"},
        )

        sm.add_state(
            "ANCHOR_TO_FIRST_FLARE",
            HitFlare(
                flare_number=1,
                outcomes=["done"],
                target_depth=-1.0,
                distance_threshold=1.5,
                completion_time_threshold=40.0,
                angle_step=0.01,
            ),
            transitions={"done": "FIRST_FLARE_TO_ANCHOR"},
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
