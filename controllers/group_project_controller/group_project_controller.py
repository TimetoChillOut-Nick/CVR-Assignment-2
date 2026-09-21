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

from project_utils import CONFIG, ROOT, world_to_grid, grid_to_world, station_coordinates, path_to_waypoints
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
    scores = []  #For printing
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


#Default movement setup ## tune this
MOVE_SPEED = 5.0 #Straightline Speed
TURN_SPEED = 4.0 #Turn Speed
BUCKET_ANGLE = (0.0, math.pi / 2, math.pi, -math.pi / 2)  #east, north, west, south


def heading_bucket(yaw):
    return round(yaw / (math.pi / 2)) % 4


def target_bucket(dx, dy):
    if abs(dx) >= abs(dy):
        return 0 if dx > 0 else 2 #east or west
    else:
        return 1 if dy > 0 else 3 #north or south


def drive_step(pose, waypoints, nav_state):
    if nav_state["phase"] == "DONE" or nav_state["index"] >= len(waypoints):
        nav_state["phase"] = "DONE"
        set_speed(0.0, 0.0)
        return

    x, y, yaw = pose
    tx, ty = waypoints[nav_state["index"]]
    dx, dy = tx - x, ty - y

    if nav_state["phase"] == "ROTATE":
        target_angle = BUCKET_ANGLE[target_bucket(dx, dy)]
        error = math.remainder(target_angle - yaw, 2 * math.pi) 

        if "turn_left" not in nav_state:
            nav_state["turn_left"] = error > 0

        arrived = error <= 0 if nav_state["turn_left"] else error >= 0
        if arrived:
            nav_state.pop("turn_left", None)
            nav_state["phase"] = "DRIVE"
        elif nav_state["turn_left"]:
            set_speed(-TURN_SPEED, TURN_SPEED)
        else:
            set_speed(TURN_SPEED, -TURN_SPEED)

    elif nav_state["phase"] == "DRIVE":
        #Check Snap to cardinal direction
        heading = heading_bucket(yaw)
        if heading == 0:
            arrived = x >= tx #east
        elif heading == 1:
            arrived = y >= ty #north
        elif heading == 2:
            arrived = x <= tx #west
        else:
            arrived = y <= ty #south

        if arrived:
            nav_state["index"] += 1
            nav_state["phase"] = "ROTATE" if nav_state["index"] < len(waypoints) else "DONE"
            set_speed(0.0, 0.0)
        else:
            set_speed(MOVE_SPEED, MOVE_SPEED)


# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    print("Group-project controller started.")
    print("Mission:", MISSION)
    print("target:", target)
    print("Stations:", [s["id"] for s in CONFIG["stations"]])
    print("Camera:", camera.getWidth(), "x", camera.getHeight())


    searched = False #run the search once, on the first valid tick
    waypoints = []
    nav_state = {"index": 0, "phase": "DONE"}

    while robot.step(timestep) != -1:
        pose = get_pose()

        if not searched:
            start_rc = world_to_grid(pose[0], pose[1])
            print(f"Starting grid location: {start_rc}")

            closest_station, path, scores = find_closest_station(GRID, start_rc, station_coordinates())

            print("Station scores (steps to reach):")
            for station_id, steps in scores:
                print(f"  {station_id}: {steps if steps is not None else 'unreachable'}")

            if closest_station is None:
                print("No reachable station found")
            else:
                print(f"Moving to closest station: {closest_station['id']}")
                waypoints = path_to_waypoints(path)
                nav_state = {"index": 0, "phase": "ROTATE" if waypoints else "DONE"}

            searched = True

        drive_step(pose, waypoints, nav_state)


if __name__ == "__main__":
    main()
