"""
3006ICT Group Project - Controller

Mission:
    Search the observation stations, identify the requested visual target,
    navigate safely, and stop at the correct target.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np
from controller import Robot

from project_utils import CONFIG, ROOT, world_to_grid, grid_to_world, station_coordinates
from Astar_helper import astar


# ------------------------------------------------------------------
# Webots setup
# ------------------------------------------------------------------
robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
camera = robot.getDevice("camera")
ps = [robot.getDevice(f"ps{i}") for i in range(8)]
gps = robot.getDevice("gps")
imu = robot.getDevice("imu")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

camera.enable(timestep)
gps.enable(timestep)
imu.enable(timestep)
for sensor in ps:
    sensor.enable(timestep)

MAX_SPEED = 10
GRID = np.load(ROOT / "maps" / "occupancy_grid.npy")
MISSION = json.loads((ROOT / "config" / "assessment_mission.json").read_text())
target = MISSION["target"]

# ------------------------------------------------------------------
# Provided low-level helpers
# ------------------------------------------------------------------
def set_speed(left, right):
    left = np.clip(left, -MAX_SPEED, MAX_SPEED)
    right = np.clip(right, -MAX_SPEED, MAX_SPEED)
    left_motor.setVelocity(float(left))
    right_motor.setVelocity(float(right))


def get_pose():
    """Return provided ground-truth-like pose (x, y, yaw)."""
    x, y, _ = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]
    return x, y, yaw


def camera_bgr():
    h, w = camera.getHeight(), camera.getWidth()
    image = np.frombuffer(camera.getImage(), np.uint8).reshape(h, w, 4)
    return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)


def proximity_values():
    return [sensor.getValue() for sensor in ps]


# ------------------------------------------------------------------
# Group implementation
# ------------------------------------------------------------------
#Find the closest(cheapest) station using astar
def find_closest_station(grid, start_rc, stations):
    best_station = None
    best_path = None
    scores = []  # (station_id, steps or None if unreachable), for printing
    #Loop through each station and save them with their score
    for station in stations:
        path = astar(grid, start_rc, station["grid"])
        steps = len(path) - 1 if path is not None else None
        scores.append((station["id"], steps))
        if path is None:
            continue  # station not reachable skip
        if best_path is None or len(path) < len(best_path):
            best_station = station
            best_path = path
    return best_station, best_path, scores

# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    print("Group-project controller started.")
    print("Mission:", MISSION)
    print("target:", target)
    print("Stations:", [s["id"] for s in CONFIG["stations"]])
    print("Camera:", camera.getWidth(), "x", camera.getHeight())


    searched = False  #Setup to run once to test the A* search before finalizing

    while robot.step(timestep) != -1:
        pose = get_pose()

        if not searched:
            ###Just some stuff to test the A*
            #Convert to grid
            start_rc = world_to_grid(pose[0], pose[1])
            print(f"Starting grid location: {start_rc}")

            #Search for closest station
            closest_station, path, scores = find_closest_station(GRID, start_rc, station_coordinates())

            print("Station scores (steps to reach):")
            for station_id, steps in scores:
                print(f"  {station_id}: {steps if steps is not None else 'unreachable'}")

            if closest_station is None:
                print("No reachable station found")
            else:
                print(f"Moving to closest station: {closest_station['id']}")
                print("Moving")

            searched = True
            ###Stops here

        set_speed(0.0, 0.0)


if __name__ == "__main__":
    main()
