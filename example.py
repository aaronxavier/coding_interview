#!/usr/bin/env python3
"""
Initializes a 20x10m rectangular arena and drives a robot along a predefined
trajectory using a ConstantVelocityExecutor (no planner needed).
"""

import threading

from pyrobosim.core import Robot, World
from pyrobosim.gui import start_gui
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.utils.path import Path
from pyrobosim.utils.pose import Pose

# Arena dimensions
W, H = 20.0, 10.0

# Waypoints: a rectangular lap around the arena perimeter with some margin
MARGIN = 1.0
TRAJECTORY = [
    Pose(x=MARGIN,     y=MARGIN),
    Pose(x=W - MARGIN, y=MARGIN),
    Pose(x=W - MARGIN, y=H - MARGIN),
    Pose(x=MARGIN,     y=H - MARGIN),
    Pose(x=MARGIN,     y=MARGIN),
]


def build_world() -> World:
    world = World(name="arena")

    # Single rectangular room covering the full arena
    footprint = [
        [0.0, 0.0],
        [W,   0.0],
        [W,   H],
        [0.0, H],
    ]
    world.add_room(name="arena", footprint=footprint, color=[0.2, 0.4, 0.6])

    executor = ConstantVelocityExecutor(linear_velocity=2.0, dt=0.05)
    robot = Robot(
        name="robot",
        radius=0.2,
        color=(0.8, 0.2, 0.0),
        path_executor=executor,
    )
    world.add_robot(robot, loc="arena", pose=TRAJECTORY[0])

    return world


def run_trajectory(world: World) -> None:
    robot = world.robots[0]
    path = Path(poses=TRAJECTORY)
    print(f"Executing trajectory with {path.num_poses} waypoints, total length {path.length:.2f} m")
    result = robot.follow_path(path, realtime_factor=1.0)
    print(f"Navigation result: {result.status.name} — {result.message or 'OK'}")


if __name__ == "__main__":
    world = build_world()

    # Run trajectory in background so the GUI can start immediately
    t = threading.Thread(target=run_trajectory, args=(world,), daemon=True)
    t.start()

    start_gui(world)
