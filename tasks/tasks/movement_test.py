#!/usr/bin/env python3

import rclpy
import smach
import math
from tasks.movement_tasks import *
from thrusters.thrusters import ThrusterControl


def main():
    rclpy.init()

    sm = smach.StateMachine(outcomes=["finish"])

    with sm:
        smach.StateMachine.add(
            "DIVE_TO_DEPTH",
            DiveToDepth(desiredDepth=-1.5, tolerance=0.05),
            transitions={"done": "ROTATE_TO_YAW"},
        )
        smach.StateMachine.add(
            "ROTATE_TO_YAW",
            RotateToYaw(desiredYaw=math.radians(90.0), tolerance=0.1),
            transitions={"done": "MOVE_STRAIGHT"},
        )
        smach.StateMachine.add(
            "MOVE_STRAIGHT",
            MoveLinearlyForTime(time_to_move=30.0, linearAcc=[0.0, 1.0, 0.0]),
            transitions={"done": "finish"},
        )

    try:
        sm.execute()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl = ThrusterControl()
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
