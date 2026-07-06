#!/usr/bin/env python3
"""Generate Smac State Lattice primitives for the Caramelo mecanum base."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


GRID_RESOLUTION = 0.02
TURNING_RADIUS = 0.0
DATE_GENERATED = "2026-07-06"

# Same heading bins used by the Nav2 16-bin omni samples. They are grid-friendly
# angles, not uniformly spaced angles.
HEADING_ANGLES = [
    0.0,
    0.4636476090008061,
    0.7853981633974483,
    1.1071487177940904,
    1.5707963267948966,
    2.0344439357957027,
    2.356194490192345,
    2.677945044588987,
    3.141592653589793,
    3.6052402625905993,
    3.9269908169872414,
    4.2487413713838835,
    4.71238898038469,
    5.176036589385496,
    5.497787143782138,
    5.81953769817878,
]

# Local-frame translations. The mecanum base can move laterally and diagonally
# without needing a nonzero turning radius.
TRANSLATION_PRIMITIVES = [
    (0.16, 0.0),
    (-0.08, 0.0),
    (0.0, 0.10),
    (0.0, -0.10),
    (0.10, 0.10),
    (0.10, -0.10),
    (-0.06, 0.08),
    (-0.06, -0.08),
]


def rounded(value: float) -> float:
    return round(float(value), 6)


def normalize_angle(angle: float) -> float:
    return angle % (2.0 * math.pi)


def shortest_delta(start: float, end: float) -> float:
    delta = normalize_angle(end - start)
    if delta > math.pi:
        delta -= 2.0 * math.pi
    return delta


def rotate_xy(x_local: float, y_local: float, heading: float) -> tuple[float, float]:
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    return (
        x_local * cos_h - y_local * sin_h,
        x_local * sin_h + y_local * cos_h,
    )


def rotation_poses(start_heading: float, end_heading: float) -> list[list[float]]:
    delta = shortest_delta(start_heading, end_heading)
    steps = max(2, int(math.ceil(abs(delta) / (math.pi / 16.0))))
    return [
        [0.0, 0.0, rounded(normalize_angle(start_heading + delta * i / steps))]
        for i in range(1, steps + 1)
    ]


def translation_poses(x_local: float, y_local: float, heading: float) -> list[list[float]]:
    x_end, y_end = rotate_xy(x_local, y_local, heading)
    distance = math.hypot(x_end, y_end)
    steps = max(1, int(math.ceil(distance / GRID_RESOLUTION)))
    return [
        [
            rounded(x_end * i / steps),
            rounded(y_end * i / steps),
            rounded(normalize_angle(heading)),
        ]
        for i in range(1, steps + 1)
    ]


def build_primitives() -> dict:
    primitives = []
    trajectory_id = 0

    for start_index, start_heading in enumerate(HEADING_ANGLES):
        for end_index, end_heading in enumerate(HEADING_ANGLES):
            if end_index == start_index:
                continue
            delta = shortest_delta(start_heading, end_heading)
            primitives.append(
                {
                    "trajectory_id": trajectory_id,
                    "start_angle_index": start_index,
                    "end_angle_index": end_index,
                    "left_turn": delta >= 0.0,
                    "trajectory_radius": TURNING_RADIUS,
                    "trajectory_length": 0.0,
                    "arc_length": 0.0,
                    "straight_length": 0.0,
                    "poses": rotation_poses(start_heading, end_heading),
                }
            )
            trajectory_id += 1

        for x_local, y_local in TRANSLATION_PRIMITIVES:
            distance = math.hypot(x_local, y_local)
            primitives.append(
                {
                    "trajectory_id": trajectory_id,
                    "start_angle_index": start_index,
                    "end_angle_index": start_index,
                    "left_turn": True,
                    "trajectory_radius": TURNING_RADIUS,
                    "trajectory_length": rounded(distance),
                    "arc_length": 0.0,
                    "straight_length": rounded(distance),
                    "poses": translation_poses(x_local, y_local, start_heading),
                }
            )
            trajectory_id += 1

    return {
        "version": 1.0,
        "date_generated": DATE_GENERATED,
        "lattice_metadata": {
            "motion_model": "omni",
            "turning_radius": TURNING_RADIUS,
            "grid_resolution": GRID_RESOLUTION,
            "stopping_threshold": 5,
            "num_of_headings": len(HEADING_ANGLES),
            "heading_angles": HEADING_ANGLES,
            "number_of_trajectories": len(primitives),
        },
        "primitives": primitives,
    }


def default_output_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "config"
        / "lattice"
        / "caramelo_omni_2cm_16bins.json"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "output",
        nargs="?",
        type=Path,
        default=default_output_path(),
        help="Output JSON path.",
    )
    args = parser.parse_args()

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as handle:
        json.dump(build_primitives(), handle, indent=2)
        handle.write("\n")
    print(f"Generated {output}")


if __name__ == "__main__":
    main()
