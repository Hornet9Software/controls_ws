import numpy as np
import pandas as pd

df = pd.read_excel("path_plan.xlsx", sheet_name=0, header=None)


df = df[:-1]
# Convert to NumPy array and find non-zero positions
non_zero_positions = df.to_numpy().nonzero()

# Map positions back to DataFrame index and columns
positions = {df.iloc[i, j]: np.array([j, -i]) for i, j in zip(*non_zero_positions)}

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


start_to_flare = positions[2] - positions[1]
flare_to_gate = positions[3] - positions[2]
gate_to_anchor = positions[4] - positions[3]

anchor_to_first_flare = positions[5] - positions[4]
first_flare_to_anchor = -anchor_to_first_flare

anchor_to_second_flare = positions[6] - positions[4]
second_flare_to_anchor = -anchor_to_second_flare

anchor_to_third_flare = positions[7] - positions[4]

third_flare_to_buckets = positions[8] - positions[7]

sequence = [
    start_to_flare,
    flare_to_gate,
    gate_to_anchor,
    anchor_to_first_flare,
    first_flare_to_anchor,
    anchor_to_second_flare,
    second_flare_to_anchor,
    anchor_to_third_flare,
    third_flare_to_buckets,
]

movement_instructions = list(
    map(lambda vec: (np.linalg.norm(vec), np.degrees(angle(vec))), sequence)
)

print(movement_instructions)
