#!/usr/bin/env python3

import rclpy
import smach
from simulation.movement_tasks_sim import *
from simulation.thrusters_sim import *


def main():
    rclpy.init()

    sm = smach.StateMachine(outcomes=["finish"])

    with sm:
        smach.StateMachine.add(
            "Depth PID Tuning",
            DiveToDepth(desiredDepth=-2.5, tolerance=0.05),
            transitions={"done": "finish"},
        )

    try:
        sm.execute()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl = ThrusterControl()
        thrusterControl.killThrusters()

    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
