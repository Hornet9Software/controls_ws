import json

import numpy as np
import pandas as pd


class PathPlanner:
    def __init__(self) -> None:
        # df = pd.read_excel("path_plan.xlsx", sheet_name=0, header=None)
        # df = df[:-1]
        # non_zero_positions = df.to_numpy().nonzero()
        # self.positions = {
        #     df.iloc[i, j]: np.array([j, -i]) for i, j in zip(*non_zero_positions)
        # }

        # self.positions = {
        #     "start": np.array([14, -25]),
        #     "orange_flare": np.array([14, -17]),
        #     "gate": np.array([14, -13]),
        #     "anchor": np.array([14, -11]),
        #     "red_flare": np.array([18, -9]),
        #     "yellow_flare": np.array([18, -11]),
        #     "blue_flare": np.array([18, -13]),
        #     "buckets": np.array([14, -5]),
        # }

        self.positions = {
            "start": np.array([-1.625, 0]),
            "orange_flare": np.array([-2.625, 6.25]),
            "gate": np.array([0, 9.5]),
            "gate_in_front": np.array([0, 10.5]),
            "anchor": np.array([9, 15.875]),
            "red_flare": np.array([11, 15.875]),
            "yellow_flare": np.array([12.75, 18.5]),
            "blue_flare": np.array([13.25, 12.625]),
            "buckets": np.array([-3.5, 23]),
        }

        self.default_flare_order = {
            "first": "yellow_flare",
            "second": "red_flare",
            "third": "blue_flare",
        }

    def compute_before_flares(self):
        start_to_orange_flare = self.positions["orange_flare"] - self.positions["start"]
        orange_flare_to_gate = self.positions["gate"] - self.positions["orange_flare"]
        gate_to_gate_in_front = self.positions["gate_in_front"] - self.positions["gate"]
        gate_in_front_to_start = (
            self.positions["start"] - self.positions["gate_in_front"]
        )
        gate_in_front_to_anchor = (
            self.positions["anchor"] - self.positions["gate_in_front"]
        )

        sequence = [
            start_to_orange_flare,
            orange_flare_to_gate,
            gate_to_gate_in_front,
            gate_in_front_to_start,
            gate_in_front_to_anchor,
        ]

        return self.compute_polar(sequence)

    def compute_flares(self, order=None):
        if order is None:
            order = self.default_flare_order
        anchor_to_first_flare = (
            self.positions[order["first"]] - self.positions["anchor"]
        )

        first_flare_to_second_flare = (
            self.positions[order["second"]] - self.positions[order["first"]]
        )

        second_flare_to_third_flare = (
            self.positions[order["third"]] - self.positions[order["second"]]
        )

        sequence = [
            anchor_to_first_flare,
            first_flare_to_second_flare,
            second_flare_to_third_flare,
        ]

        return self.compute_polar(sequence)

    def compute_polar(self, vecs):
        return list(map(lambda vec: [np.linalg.norm(vec), self.angle(vec)], vecs))

    def angle(self, vec):
        x = vec[0]
        y = vec[1]

        xa = abs(x)
        ya = abs(y)
        basic_angle = np.arctan2(ya, xa)

        theta = 0

        if x >= 0 and y >= 0:
            theta = basic_angle
        elif x < 0 and y >= 0:
            theta = np.pi - basic_angle
        elif x < 0 and y < 0:
            theta = np.pi + basic_angle
        else:
            theta = (2 * np.pi) - basic_angle

        return theta - (np.pi / 2)
