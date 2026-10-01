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


def detect_target(frame):
    """Placeholder for real target detection; always reports not-found for now."""
    return False


# ------------------------------------------------------------------
# Safety / interrupt layer
#
# e-puck ps0..ps7 layout (standard Webots e-puck, sensors numbered
# clockwise from the front-right):
#   ps0 front-right   ps1 right        ps2 right-rear
#   ps3 rear-right    ps4 rear-left    ps5 left-rear
#   ps6 left          ps7 front-left
#
# ------------------------------------------------------------------
FRONT_IDX = (0, 1, 6, 7)        # front-right, right, left, front-left
REAR_IDX = (2, 3, 4, 5)         # right-rear, rear-right, rear-left, left-rear
RIGHT_IDX = (0, 1, 2)
LEFT_IDX = (7, 6, 5)

PS_TRIGGER = 85.0                # sensors pickup background noise between 59 to 75

RECOVERY_MOVE_S = 0.5            # seconds spent driving straight clear
RECOVERY_TURN_S = 0.4            # seconds spent turning clear afterwards
RECOVERY_TURN_MAX_S = 1.2        # safety valve so TURN can't run forever


def is_unsafe(prox):
    """True if any proximity sensor is inside safety margin."""
    return any(v > PS_TRIGGER for v in prox)


def start_recovery(prox):
    """Decide which way to escape: straight-line direction away from whichever
    end (front/rear) triggered, and which way to turn once clear, away from
    whichever side (left/right) triggered. Without this, an obstacle behind
    the robot would make it reverse straight into it.
    """
    front = max(prox[i] for i in FRONT_IDX)
    rear = max(prox[i] for i in REAR_IDX)
    right = max(prox[i] for i in RIGHT_IDX)
    left = max(prox[i] for i in LEFT_IDX)
    escape = "BACKUP" if front >= rear else "FORWARD"   # move away from closer end
    turn_left = right >= left                            # turn away from closer side
    return {"phase": escape, "timer": 0, "turn_left": turn_left}


def recovery_step(rec_state, prox):
    """One tick of the escape manoeuvre (move clear, then turn away).

    Returns True once the manoeuvre is finished and it is safe to hand
    control back to the mission state machine.
    """
    move_steps = max(1, int(RECOVERY_MOVE_S * 1000 / timestep))
    turn_steps = max(1, int(RECOVERY_TURN_S * 1000 / timestep))
    turn_steps_max = max(turn_steps, int(RECOVERY_TURN_MAX_S * 1000 / timestep))

    if rec_state["phase"] in ("BACKUP", "FORWARD"):
        speed = -MOVE_SPEED if rec_state["phase"] == "BACKUP" else MOVE_SPEED
        set_speed(speed, speed)
        rec_state["timer"] += 1
        if rec_state["timer"] >= move_steps or not is_unsafe(prox):
            rec_state["phase"] = "TURN"
            rec_state["timer"] = 0
        return False

    if rec_state["phase"] == "TURN":
        if rec_state["turn_left"]:
            set_speed(-TURN_SPEED, TURN_SPEED)
        else:
            set_speed(TURN_SPEED, -TURN_SPEED)
        rec_state["timer"] += 1
        if rec_state["timer"] >= turn_steps and not is_unsafe(prox):
            return True
        if rec_state["timer"] >= turn_steps_max:   # don't spin forever
            return True
        return False

    return True


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


def search_step(pose, search_state):
    """Spin in place scanning for the target. Returns True once a full revolution completes."""
    yaw = pose[2]
    delta = math.remainder(yaw - search_state["prev_yaw"], 2 * math.pi)
    search_state["accumulated"] += abs(delta)
    search_state["prev_yaw"] = yaw

    if search_state["accumulated"] >= 2 * math.pi:
        set_speed(0.0, 0.0)
        return True

    set_speed(-TURN_SPEED, TURN_SPEED)
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

    #Interrupt wrapper: while safe -> run mission state machine as
    #normal; while unsafe (proximity_values() >= PS_TRIGGER) -> suspend
    #state machine and run escape manoeuvre instead. recovery_state is
    #None whenever not currently recovering.
    recovery_state = None

    while robot.step(timestep) != -1:
        pose = get_pose()
        prox = proximity_values()

        if is_unsafe(prox):
            if recovery_state is None:
                print(f"!! obstacle inside safety margin (ps={[round(v) for v in prox]}); "
                      f"pausing mission, rerouting")
                recovery_state = start_recovery(prox)
            if recovery_step(recovery_state, prox):
                recovery_state = None
                # pose/heading changed during escape manoeuvre. makes
                # state machine re-orient itself before it acts again
                if state == "NAVIGATION":
                    need_path = True
                elif state == "SEARCH" and search_state is not None:
                    search_state["prev_yaw"] = get_pose()[2]
                print("Clear again, resuming mission")
            continue   # skip state machine this tick

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
                waypoints = path_to_waypoints(path)
                nav_state = {"index": 0, "phase": "ROTATE" if waypoints else "DONE"}

            drive_step(pose, waypoints, nav_state)
            if nav_state["phase"] == "DONE":
                print(f"Arrived at {current_station['id']}, searching")
                state = "SEARCH"
                search_state = {"accumulated": 0.0, "prev_yaw": pose[2]}

        #Lucky this is the search state so this is where you would put the object detection in#################################
        #When it detects the object get it to switch to the 
        elif state == "SEARCH":
            if detect_target(camera_bgr()):
                print(f"Target found at {current_station['id']}")
                state = "FOUND"
                set_speed(0.0, 0.0)
            elif search_step(pose, search_state):
                print(f"No target at {current_station['id']} station dropped")
                remaining_stations = [s for s in remaining_stations if s["id"] != current_station["id"]]
                state = "NAVIGATION"
                need_path = True

        else:  #FOUND or DONE
            set_speed(0.0, 0.0)


if __name__ == "__main__":
    main()
