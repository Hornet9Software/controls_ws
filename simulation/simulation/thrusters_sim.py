#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# Dummy class, ignores anything sent to setThrusters
class ThrusterControl:
    def __init__(self):
        pass

    def setThrusters(self, thrustValues):
        pass

    def killThrusters(self):
        pass


# Thrust allocator which publishes lin and angular acc for simulation purposes

import os.path
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import lsq_linear

# fmt: off
thruster_positions = np.array(
    [
        [-0.22, 0.238, -0.054],     # Front Left
        [0.22, 0.238, -0.054],      # Front Right
        [-0.234, -0.001, -0.107],   # Middle Left
        [0.234, -0.001, -0.107],    # Middle Right
        [-0.22, -0.217, -0.054],    # Rear Left
        [0.22, -0.217, -0.054],     # Rear Right
    ]
)

thruster_directions = np.array(
    [
        [1, 1, 0],                  # Front Left
        [-1, 1, 0],                 # Front Right
        [0, 0, 1],                  # Middle Left
        [0, 0, 1],                  # Middle Right
        [-1, 1, 0],                 # Rear Left
        [1, 1, 0],                  # Rear Right
    ]
)

thruster_biases = np.array([1.0,    # Front Left
                            1.0,    # Front Right
                            1.0,    # Middle Left
                            1.0,    # Middle Right
                            1.0,    # Rear Left
                            1.0])   # Rear Right
# fmt: on

thruster_directions = thruster_directions / np.linalg.norm(
    thruster_directions, keepdims=True, axis=1
)

I = np.array([0.205, 0.141, 0.205])

rosThrustMapPath = "src/controls_ws/controls_core/controls_core/thrust_map.csv"
wsThrustMapPath = "controls_core/thrust_map.csv"  # For debugging with test.ipynb
print(Path.cwd())

if os.path.exists(rosThrustMapPath):
    thrust_map = pd.read_csv(rosThrustMapPath).values
else:
    thrust_map = pd.read_csv(wsThrustMapPath).values


class ThrustAllocator(Node):
    def __init__(
        self,
        thruster_positions=thruster_positions,
        thruster_directions=thruster_directions,
        mass=18.0,
        I=I,
        thrust_map=thrust_map,
    ):
        super().__init__("thrust_allocator_sim")
        self.thruster_positions = thruster_positions
        self.thruster_directions = thruster_directions
        self.mass = mass
        self.I = I
        self.thrust_map = thrust_map

        self.unit_torque = np.cross(self.thruster_positions, self.thruster_directions)
        self.parameters = np.concatenate(
            (self.thruster_directions.T, self.unit_torque.T)
        )

        self.acc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.accPub = self.create_publisher(Float32MultiArray, "/sim/acc", 10)

    def getThrusts(self, linear_accelerations, angular_accelerations):
        linear_accelerations = np.array(linear_accelerations)
        angular_accelerations = np.array(angular_accelerations)

        self.expected_force = self.mass * linear_accelerations
        self.expected_torque = self.I * angular_accelerations
        self.goal = np.concatenate((self.expected_force, self.expected_torque))

        lb = np.min(self.thrust_map[:, 0]) + 0.1
        ub = np.max(self.thrust_map[:, 0]) - 0.1

        thrust_newtons = lsq_linear(self.parameters, self.goal, bounds=(lb, ub)).x
        return thrust_newtons

    def getThrustPWMs(self, linear_accelerations, angular_accelerations):
        self.acc = linear_accelerations + angular_accelerations
        msg = Float32MultiArray()
        msg.data = self.acc
        self.accPub.publish(msg)

        thrust_newtons = self.getThrusts(linear_accelerations, angular_accelerations)
        thrust_biased = thrust_newtons * thruster_biases

        thrust_converted = self.thrust_map[
            np.searchsorted(self.thrust_map[:, 0], thrust_biased, side="left"), 1
        ].astype(int)

        return thrust_converted
