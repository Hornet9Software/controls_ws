import math

import rclpy
import smach
from tasks.movement_tasks import *
from thrusters.thrusters import ThrusterControl


def main():
    rclpy.init()

    sm = smach.StateMachine(outcomes=["finish"])

    with sm:
        smach.StateMachine.add(
            "DIVE_TO_DEPTH",
            DiveToDepth(targetDepth=-0.15, tolerance=0.05, setYaw=1.57),
            # transitions={"done": "ROTATE_TO_YAW"},
            transitions={"done": "finish"},
            # transitions={"done": "MOVE_STRAIGHT"},
        )
        # smach.StateMachine.add(
        #     "ROTATE_TO_YAW",
        #     RotateToYaw(targetYaw=math.radians(90.0), tolerance=0.1),
        #     transitions={"done": "MOVE_STRAIGHT"},
        # )
        # smach.StateMachine.add(
        #     "MOVE_STRAIGHT",
        #     MoveLinearlyForTime(timeToMove=30.0, yAcc=1.0, desiredDepth=-1.0),
        #     transitions={"done": "finish"},
        # )

    try:
        sm.execute()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl = ThrusterControl()
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
