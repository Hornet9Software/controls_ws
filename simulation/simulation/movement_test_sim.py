import math

import rclpy
import smach
import smach_ros
from tasks.movement_tasks import *


def main():
    rclpy.init()

    sm = smach.StateMachine(outcomes=["finish"])

    with sm:
        # smach.StateMachine.add(
        #     "DIVE_TO_DEPTH",
        #     DiveToDepth(targetDepth=-1.0, tolerance=0.05, setYaw=1.57),
        #     transitions={"done": "ROTATE_TO_YAW"},
        #     # transitions={"done": "finish"},
        #     # transitions={"done": "MOVE_STRAIGHT"},
        # )
        # smach.StateMachine.add(
        #     "ROTATE_TO_YAW",
        #     RotateToYaw(targetYaw=math.radians(95.0), tolerance=0.02, setDepth=-1.0),
        #     transitions={"done": "MOVE_STRAIGHT"},
        # )
        # smach.StateMachine.add(
        #     "MOVE_STRAIGHT",
        #     MoveStraightForTime(timeToMove=0.1, setDepth=-1.0, setYaw=1.57),
        #     transitions={"done": "finish"},
        # )
        smach.StateMachine.add(
            "MOVE_TO_GATE",
            MoveToGate(
                tolerance=0.05,
                bearingControl=True,
                lateralControl=False,
                lateralDirection="right",
                distanceControl=False,
                setDepth=-1.0,
            ),
            transitions={"done": "finish"},
        )

    sis = smach_ros.IntrospectionServer("smach_server", sm, "/SM_ROOT")
    sis.start()

    try:
        sm.execute()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.try_shutdown()
        sis.stop()


if __name__ == "__main__":
    main()
