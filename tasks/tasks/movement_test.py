#!/usr/bin/env python3

import rclpy
import smach
from tasks.movement_tasks import MoveToPoseGlobalTask


def main():
    rclpy.init()

    sm = smach.StateMachine(outcomes=["finish"])

    with sm:
        smach.StateMachine.add(
            "MoveForward",
            MoveToPoseGlobalTask(0.0, 5.0, 0.0, 0.0, 0.0, 0.0),
            transitions={"done": "finish"},
        )

    sm.execute()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
