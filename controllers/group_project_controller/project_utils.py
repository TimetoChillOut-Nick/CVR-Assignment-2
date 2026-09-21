"""Coordinate helpers for the 3006ICT group-project world."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONFIG = json.loads((ROOT / "config" / "project_config.json").read_text())

X_MIN = CONFIG["arena"]["x_min"]
Y_MAX = CONFIG["arena"]["y_max"]
RES = CONFIG["arena"]["resolution"]


def world_to_grid(x, y):
    """World (x, y) -> occupancy-grid (row, col)."""
    col = int((x - X_MIN) / RES)
    row = int((Y_MAX - y) / RES)
    return row, col


def grid_to_world(row, col):
    """Occupancy-grid cell -> world coordinate at the cell centre."""
    x = X_MIN + (col + 0.5) * RES
    y = Y_MAX - (row + 0.5) * RES
    return x, y


def station_by_id(station_id):
    return next(s for s in CONFIG["stations"] if s["id"] == station_id)


def start_by_id(start_id):
    return next(s for s in CONFIG["starts"] if s["id"] == start_id)


"""Coordinate Helpers built by the team"""
#Gen a list of all stations
def station_coordinates():
    coords = []
    for s in CONFIG["stations"]:
        x, y = s["observe"]
        row, col = world_to_grid(x, y)
        coords.append({"id": s["id"], "world": (x, y), "grid": (row, col)})
    return coords


#Turn astar path into waypoints at turning pointss
def path_to_waypoints(path):
    if not path or len(path) < 2:
        return []  #if no path or at desination

    waypoints = []
    prev_dir = None
    for i in range(1, len(path)):
        r0, c0 = path[i - 1]
        r1, c1 = path[i]
        direction = (r1 - r0, c1 - c0)
        if prev_dir is not None and direction != prev_dir:
            #direction changed stored here
            waypoints.append(grid_to_world(*path[i - 1]))
        prev_dir = direction

    waypoints.append(grid_to_world(*path[-1]))  #always end at goal
    return waypoints