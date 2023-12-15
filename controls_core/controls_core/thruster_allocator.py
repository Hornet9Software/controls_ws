#!/usr/bin/env python3

import numpy as np
import pandas as pd
import math
from scipy.optimize import lsq_linear

thruster_positions = np.array(
    [
        [-0.22, 0.22, -0.22, 0.22, -0.234, 0.234],
        [0.238, 0.238, -0.217, -0.217, -0.001, -0.001],
        [-0.054, -0.054, -0.054, -0.054, -0.107, -0.107],
    ]
)

thruster_directions = np.array(
    [
        [0.707, -0.707, -0.707, 0.707, 0.0, 0.0],
        [0.707, 0.707, 0.707, 0.707, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0, 1.0],
    ]
)

I = np.array([0.205, 0.141, 0.205])

thrust_map = pd.read_csv("thrust_map.csv").values


class ThrustAllocator:
    def __init__(
        self,
        thruster_positions=thruster_positions,
        thruster_directions=thruster_directions,
        mass=18.0,
        I=I,
        thrust_map=thrust_map,
    ):
        self.thruster_positions = thruster_positions
        self.thruster_directions = thruster_directions
        self.mass = mass
        self.I = I
        self.thrust_map = thrust_map

        self.unit_torque = np.cross(
            self.thruster_positions.T, self.thruster_directions.T
        ).T
        self.parameters = np.concatenate((self.thruster_directions, self.unit_torque))

    def solve(self, linear_accelerations, angular_accelerations):
        linear_accelerations[2] = linear_accelerations[2]
        self.expected_force = self.mass * linear_accelerations
        self.expected_torque = self.I * angular_accelerations
        self.goal = np.concatenate((self.expected_force, self.expected_torque))

        lb = np.min(self.thrust_map[:, 0]) + 0.1
        ub = np.max(self.thrust_map[:, 0]) - 0.1

        thrust_newtons = lsq_linear(self.parameters, self.goal, bounds=(lb, ub)).x
        thrust_converted = self.thrust_map[
            np.searchsorted(self.thrust_map[:, 0], thrust_newtons, side="left"), 1
        ].astype(int)

        return thrust_converted


def test():
    solver = ThrustAllocator()
    lin_acc = np.array([1.0, 0.0, 0.0])
    angular_acc = np.array([0.0, 0.0, 0.0])

    print(solver.solve(lin_acc, angular_acc))


if __name__ == "__main__":
    test()
