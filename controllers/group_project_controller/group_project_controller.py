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
from smoothing_helper import smooth_waypoints
from safety_helper import is_unsafe, start_recovery, recovery_step
from vision_helper import TargetDetector, MIN_INLIERS
from run_log import new_run_log, add_time, add_station, print_summary


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

#puck wheel motor max velocity (rad/s)
MAX_SPEED = 6.28
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
MOVE_SPEED = 6.0 #Straightline Speed
TURN_SPEED = 4.0 #Turn Speed

#Smooth the A* path, False uses the A* turning points
SMOOTH_PATHS = True

#Heading error (rad) to turn on the spot, and to drive again
ROTATE_ERROR = 0.5
ALIGNED_ERROR = 0.1
#Steering strength while driving
STEER_GAIN = 4.0
#Distance (m) that counts as reaching a waypoint
WAYPOINT_TOL = 0.05


#Turn on the spot if well off course, otherwise steer while driving
def drive_step(pose, waypoints, nav_state):
    if nav_state["phase"] == "DONE" or nav_state["index"] >= len(waypoints):
        nav_state["phase"] = "DONE"
        set_speed(0.0, 0.0)
        return

    x, y, yaw = pose
    tx, ty = waypoints[nav_state["index"]]
    dx, dy = tx - x, ty - y

    if math.hypot(dx, dy) < WAYPOINT_TOL:
        nav_state["index"] += 1
        if nav_state["index"] >= len(waypoints):
            nav_state["phase"] = "DONE"
            set_speed(0.0, 0.0)
        return

    error = math.remainder(math.atan2(dy, dx) - yaw, 2 * math.pi)

    if nav_state["phase"] == "ROTATE" and abs(error) < ALIGNED_ERROR:
        nav_state["phase"] = "DRIVE"
    elif nav_state["phase"] == "DRIVE" and abs(error) > ROTATE_ERROR:
        nav_state["phase"] = "ROTATE"

    if nav_state["phase"] == "ROTATE":
        turn = TURN_SPEED if error > 0 else -TURN_SPEED
        set_speed(-turn, turn)
    else:
        steer = STEER_GAIN * error
        set_speed(MOVE_SPEED - steer, MOVE_SPEED + steer)


#Back away slowly so more of the poster fits in view
SEARCH_BACKUP_SPEED = 2.0
SEARCH_BACKUP_M = 0.30


def new_search_state(station):
    return {"phase": "TURN", "yaw": station["yaw"], "start": None}


#Face the station then back away, True once backed up the full distance
def search_step(pose, search_state):
    if search_state["phase"] == "DONE":
        set_speed(0.0, 0.0)
        return True

    if search_state["phase"] == "TURN":
        error = math.remainder(search_state["yaw"] - pose[2], 2 * math.pi)

        if "turn_left" not in search_state:
            search_state["turn_left"] = error > 0

        arrived = error <= 0 if search_state["turn_left"] else error >= 0
        if arrived:
            search_state["phase"] = "BACKUP"
            search_state["start"] = (pose[0], pose[1])
            set_speed(0.0, 0.0)
        elif search_state["turn_left"]:
            set_speed(-TURN_SPEED, TURN_SPEED)
        else:
            set_speed(TURN_SPEED, -TURN_SPEED)
        return False

    sx, sy = search_state["start"]
    if math.hypot(pose[0] - sx, pose[1] - sy) >= SEARCH_BACKUP_M:
        set_speed(0.0, 0.0)
        return True

    set_speed(-SEARCH_BACKUP_SPEED, -SEARCH_BACKUP_SPEED)
    return False


# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    print("Group-project controller started.")
    print("Mission:", MISSION)
    print("target:", target)
    print("Stations:", [s["id"] for s in CONFIG["stations"]])
    print("Camera:", camera.getWidth(), "x", camera.getHeight())

    #Load the target 
    detector = TargetDetector(target)

    #State machine:
    #NAVIGATION Drive to closest Station
    #SEARCH Search for the detected target
    #Found Move to the detected target
    state = "NAVIGATION"
    need_path = True
    remaining_stations = station_coordinates()
    current_station = None
    waypoints = []
    nav_state = {"index": 0, "phase": "DONE"}
    search_state = None

    #None when not escaping an obstacle
    recovery_state = None
    run_log = new_run_log()

    while robot.step(timestep) != -1:
        pose = get_pose()
        prox = proximity_values()
        add_time(run_log, state, timestep / 1000)

        #Safety interrupt, pauses the state machine until escape finishes
        #Skipped once stopped so the bot stays on its final spot
        stopped = state in ("FOUND", "DONE")
        if not stopped and (recovery_state is not None or is_unsafe(prox)):
            if recovery_state is None:
                print(f"!! obstacle inside safety margin (ps={[round(v) for v in prox]}); "
                      f"pausing mission, rerouting")
                recovery_state = start_recovery(prox)
            done, left, right = recovery_step(recovery_state, prox, timestep, MOVE_SPEED, TURN_SPEED)
            set_speed(left, right)
            if done:
                recovery_state = None
                #Pose changed so replan / turn back to the station
                if state == "NAVIGATION":
                    need_path = True
                elif state == "SEARCH" and search_state["phase"] == "BACKUP":
                    #Something behind stopped the backup, finish this station
                    search_state["phase"] = "DONE"
                elif state == "SEARCH":
                    search_state = new_search_state(current_station)
                    detector.reset()
                elif state == "RETURN":
                    nav_state = {"index": 0, "phase": "ROTATE"}
                print("Clear again, resuming mission")
            continue

        #Navigation State for A* travel to each station
        if state == "NAVIGATION":
            if need_path:
                start_rc = world_to_grid(pose[0], pose[1])
                current_station, path, scores = find_closest_station(GRID, start_rc, remaining_stations)

                print("Station Distance Scores:")
                for station_id, steps in scores:
                    print(f"  {station_id}: {steps if steps is not None else 'unreachable'}")

                need_path = False
                if current_station is None:
                    print("No reachable stations left")
                    state = "DONE"
                    set_speed(0.0, 0.0)
                    continue

                print(f"Moving to station: {current_station['id']}")
                waypoints = smooth_waypoints(GRID, path) if SMOOTH_PATHS else path_to_waypoints(path)
                nav_state = {"index": 0, "phase": "ROTATE" if waypoints else "DONE"}

            drive_step(pose, waypoints, nav_state)
            if nav_state["phase"] == "DONE":
                print(f"Arrived at {current_station['id']}, searching")
                state = "SEARCH"
                search_state = new_search_state(current_station)
                detector.reset()

        #Lucky this is the search state so this is where you would put the object detection in#################################
        #When it detects the object get it to switch to the 
        elif state == "SEARCH":
            #Only scan while backing away, not while turning
            scanning = search_state["phase"] == "BACKUP"
            found = scanning and detector.update(camera_bgr())
            best = (f"best target score {detector.best_target} (need {MIN_INLIERS}), "
                    f"best other {detector.best_other} ({detector.best_other_name})")
            #Station search ends on a match or once fully backed up
            finished = found or search_step(pose, search_state)
            if finished:
                add_station(run_log, current_station["id"], detector.best_target,
                            detector.best_other, detector.best_other_name, found)

            if found:
                print(f"Target found at {current_station['id']}, {best}, returning to observe point")
                state = "RETURN"
                waypoints = [current_station["world"]]
                nav_state = {"index": 0, "phase": "ROTATE"}
            elif finished:
                print(f"No target at {current_station['id']}, {best}, station dropped")
                remaining_stations = [s for s in remaining_stations if s["id"] != current_station["id"]]
                state = "NAVIGATION"
                need_path = True

        #Return state, drive back to the observe point after a match
        elif state == "RETURN":
            drive_step(pose, waypoints, nav_state)
            if nav_state["phase"] == "DONE":
                ox, oy = current_station["world"]
                error = math.hypot(pose[0] - ox, pose[1] - oy)
                print(f"Stopped at {current_station['id']}, {error:.3f} m from the observe point")
                run_log["final_error"] = error
                state = "FOUND"

        else:  #FOUND or DONE
            set_speed(0.0, 0.0)
            print_summary(run_log, robot.getTime())


if __name__ == "__main__":
    main()
