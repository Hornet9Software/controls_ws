import numpy as np
import pandas as pd
import json

df = pd.read_excel("path_plan.xlsx", sheet_name=0, header=None)


df = df[:-1]

non_zero_positions = df.to_numpy().nonzero()

positions = {df.iloc[i, j]: np.array([j, -i]) for i, j in zip(*non_zero_positions)}

positions = {
    1: np.array([, ]),
    2: np.array([, ]),
    3: np.array([, ]),
    4: np.array([, ]),
    5: np.array([, ]),
    6: np.array([, ]),
    7: np.array([, ]),
    8: np.array([, ]),
}

print(positions)


def angle(vec):
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


start_to_orange_flare = positions[2] - positions[1]
orange_flare_to_gate = positions[3] - positions[2]
gate_to_anchor = positions[4] - positions[3]

anchor_to_first_flare = positions[5] - positions[4]
first_flare_to_anchor = -anchor_to_first_flare

anchor_to_second_flare = positions[6] - positions[4]
second_flare_to_anchor = -anchor_to_second_flare

anchor_to_third_flare = positions[7] - positions[4]

third_flare_to_buckets = positions[8] - positions[7]

sequence = [
    start_to_orange_flare,
    orange_flare_to_gate,
    gate_to_anchor,
    anchor_to_first_flare,
    first_flare_to_anchor,
    anchor_to_second_flare,
    second_flare_to_anchor,
    anchor_to_third_flare,
    third_flare_to_buckets,
]

movement_instructions = list(
    map(lambda vec: [np.linalg.norm(vec), angle(vec)], sequence)
)

with open("path.json", "w") as f:
    json.dump(movement_instructions, f, indent=4)

print(movement_instructions)
