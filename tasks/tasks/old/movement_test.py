import math

import rclpy
import smach
from tasks.movement_tasks import *
from thrusters.thrusters import ThrusterControl


def main():
    rclpy.init()

    sm = smach.StateMachine(outcomes=["finish"])

    with sm:
        # smach.StateMachine.add(
        #     "DIVE_TO_DEPTH",
        #     DiveToDepth(targetDepth=-1.0, tolerance=0.05, setYaw=1.57),
        #     # transitions={"done": "ROTATE_TO_YAW"},
        #     transitions={"done": "finish"},
        # )
        smach.StateMachine.add(
            "ROTATE_TO_YAW",
            RotateToYaw(targetYaw=math.radians(95.0), tolerance=0.02, setDepth=-1.0),
            transitions={"done": "finish"},
        )
        # smach.StateMachine.add(
        #     "MOVE_STRAIGHT",
        #     MoveStraightForTime(timeToMove=0.1, setDepth=-1.0, setYaw=1.57),
        #     transitions={"done": "MOVE_TO_GATE"},
        # )
        # smach.StateMachine.add(
        #     "MOVE_TO_GATE",
        #     MoveToGate(
        #         tolerance=0.05,
        #         bearingControl=True,
        #         lateralControl=False,
        #         lateralDirection="right",
        #         distanceControl=False,
        #         setDepth=-1.0,
        #     ),
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
