#!/usr/bin/env python3

import rclpy
import smach
from tasks.movement_tasks import *
from thrusters.thrusters import ThrusterControl


def main():
    rclpy.init()

    sm = smach.StateMachine(outcomes=["finish"])

    with sm:
        smach.StateMachine.add(
            "Steer To Gate",
            RotateToObject(objectName="gate", tolerance=1.0),
            transitions={"done": "finish"},
        )

    try:
        sm.execute()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()

    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
