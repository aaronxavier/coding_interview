#!/usr/bin/env python3
"""
20x10m arena with three colored objects (red, blue, green) at random locations.
A behavior logic drives the robot to search for the green object by visiting
each table in turn until it is found.

The control loop exits when the green is found or all locations are exhausted).
"""

import random
import threading

from pyrobosim.core import Robot, World
from pyrobosim.core.objects import Object
from pyrobosim.gui import start_gui
from pyrobosim.navigation.world_graph import WorldGraphPlanner
from pyrobosim.navigation.execution import ConstantVelocityExecutor
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from pyrobosim.planning.actions import ExecutionStatus

# ---------------------------------------------------------------------------
# World constants
# ---------------------------------------------------------------------------
W, H = 20.0, 10.0
NUM_TABLES = 9
ROBOT_RADIUS = 0.2
MARGIN = 1.5          # min distance between table centre and walls
TABLE_SPACING = 2.5   # min separation between table centres

# ---------------------------------------------------------------------------
# World builder
# ---------------------------------------------------------------------------

def _random_table_poses(n: int, rng: random.Random) -> list[Pose]:
    """Return n non-overlapping table poses inside the arena."""
    poses: list[Pose] = []
    max_tries = 500
    for _ in range(n):
        for _ in range(max_tries):
            x = rng.uniform(MARGIN, W - MARGIN)
            y = rng.uniform(MARGIN, H - MARGIN)
            candidate = Pose(x=x, y=y)
            if all(
                ((candidate.x - p.x) ** 2 + (candidate.y - p.y) ** 2) ** 0.5 >= TABLE_SPACING
                for p in poses
            ):
                poses.append(candidate)
                break
    return poses


def build_world(seed: int = 42) -> World:
    rng = random.Random(seed)

    data = get_data_folder()
    world = World(name="arena")
    world.set_metadata(
        locations=[data / "example_location_data.yaml"],
        objects=[data / "example_object_data.yaml"],
    )

    # Single open rectangular room
    world.add_room(
        name="arena",
        footprint=[[0, 0], [W, 0], [W, H], [0, H]],
        color=[0.15, 0.35, 0.55],
    )

    # Place NUM_TABLES tables at random positions
    table_poses = _random_table_poses(NUM_TABLES, rng)
    for i, pose in enumerate(table_poses):
        world.add_location(
            name=f"table{i}",
            category="table",
            parent="arena",
            pose=pose,
        )

    # Shuffle which tables get the colored objects
    object_specs = [
        ("red_obj",   "apple", (1.0, 0.0, 0.0)),
        ("blue_obj",  "apple", (0.0, 0.2, 1.0)),
        ("green_obj", "apple", (0.0, 0.85, 0.1)),
    ]
    chosen_tables = rng.sample([f"table{i}_tabletop" for i in range(NUM_TABLES)], len(object_specs))
    rng.shuffle(object_specs)

    for (obj_name, category, color), parent in zip(object_specs, chosen_tables):
        world.add_object(name=obj_name, category=category, color=color, parent=parent)

    # Robot with WorldGraphPlanner + ConstantVelocityExecutor
    planner = WorldGraphPlanner(
        max_connection_dist=15.0,
        collision_check_step_dist=0.025,
        compress_path=True,
    )
    executor = ConstantVelocityExecutor(linear_velocity=3.0, dt=0.05)
    robot = Robot(
        name="robot",
        radius=ROBOT_RADIUS,
        color=(0.9, 0.5, 0.0),
        path_planner=planner,
        path_executor=executor,
    )
    start_pose = Pose(x=W / 2, y=H / 2)
    world.add_robot(robot, loc="arena", pose=start_pose)

    return world


# ---------------------------------------------------------------------------
# Search task
# ---------------------------------------------------------------------------

def run_search(world: World) -> None:
    robot = world.robots[0]

    # Search loop: Your code here should implement the logic to visit each table in turn, check for the green object, and exit when found or exhausted.


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    world = build_world(seed=42)

    t = threading.Thread(target=run_search, args=(world,), daemon=True)
    t.start()

    start_gui(world)
