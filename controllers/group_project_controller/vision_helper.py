"""
Target detection using ORB feature matching between the target
reference image and the robot camera frame.
"""

import cv2

from project_utils import ROOT

#test6
MATCH_DISTANCE_THRESHOLD = 60
#^^^


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


#Called every tick in SEARCH, return True when the target is in view
def detect_target(frame, reference_descriptors):
    #test4/5
    frame_keypoints, frame_descriptors = find_features(frame)
    if frame_descriptors is None:
        return False

    matches, good_matches = match_features(reference_descriptors, frame_descriptors)
    print("Matches:", len(matches))
    print("Good matches:", len(good_matches))
    if len(matches) > 0:
        print("Best match distance:", matches[0].distance)

    #TODO decide when there are enough good matches to count as found
    return False
