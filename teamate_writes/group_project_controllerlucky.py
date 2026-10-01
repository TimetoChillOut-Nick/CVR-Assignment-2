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

from project_utils import CONFIG, ROOT, world_to_grid, grid_to_world


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
    
    
#test6
MATCH_DISTANCE_THRESHOLD = 60
#^^^
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
# TO DO
#grayscale
def process_camera_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return gray
    
#function to load target image    
def load_target_image(target_name):
    target_path = ROOT / "textures" / f"target_{target_name}.png"
    image = cv2.imread(str(target_path))
    return image
    
#feature function
def find_features(image):
    #convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #create ORB, store results in orb
    orb = cv2.ORB_create()
    #detection and description
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    return keypoints, descriptors

#feature matching
#take features from image image and current robot camera
def match_features(reference_descriptors, frame_descriptors):
    #brute force matcher compare descriptors on images and find best match
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    #performe matching
    matches = matcher.match(reference_descriptors, frame_descriptors)
    #use .distance value for sort
    matches = sorted(matches, key=lambda match: match.distance)
    #keep matchest that below distance threshold
    good_matches = [
    match for match in matches
        if match.distance < MATCH_DISTANCE_THRESHOLD
    ]

    return matches, good_matches

# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------
def main():
    print("Group-project controller started.")
    print("Mission:", MISSION)
    print("target:", target)
    print("Stations:", [s["id"] for s in CONFIG["stations"]])
    print("Camera:", camera.getWidth(), "x", camera.getHeight())

    #image test2
    reference = load_target_image(target)
    print("Target reference:", reference.shape)
    
    #test3
    reference_keypoints, reference_descriptors = find_features(reference)
    print("Reference keypoints:", len(reference_keypoints))
    print("Descriptor shape:", reference_descriptors.shape)

    while robot.step(timestep) != -1:
        pose = get_pose()
        # TO DO
        
        #camera test  
        frame = camera_bgr()

        #test4/5
        frame_keypoints, frame_descriptors = find_features(frame)
        if frame_descriptors is not None:
            matches, good_matches = match_features(
                reference_descriptors,
                frame_descriptors
            )
              
            print("Matches:", len(matches))
            print("Good matches:", len(good_matches))
        
            if len(matches) > 0:
                print("Best match distance:", matches[0].distance)
        
        set_speed(0.0, 0.0)


if __name__ == "__main__":
    main()
