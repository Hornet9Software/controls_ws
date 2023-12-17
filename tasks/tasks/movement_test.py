#!/usr/bin/env python3

import rclpy
import smach
from tasks.movement_tasks import MoveToPoseSimple


def main():
    rclpy.init()

    sm = smach.StateMachine(outcomes=["finish"])

    with sm:
        smach.StateMachine.add(
            "MoveForward",
            MoveToPoseSimple(0.0, 5.0, 0.0, 0.0, 0.0, 0.0),
            transitions={"done": "finish"},
        )

    sm.execute()

    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
