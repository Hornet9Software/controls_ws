#!/usr/bin/env python3

import rclpy
import smach
from tasks.movement_tasks_simulation import *


def main():
    rclpy.init()

    sm = smach.StateMachine(outcomes=["finish"])

    with sm:
        smach.StateMachine.add(
            "Depth PID Tuning",
            DiveToDepth(desiredDepth=1.5, tolerance=0.05),
            transitions={"done": "finish"},
        )

    try:
        sm.execute()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        clpy.try_shutdown()


if __name__ == "__main__":
    main()
