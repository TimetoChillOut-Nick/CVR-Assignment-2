"""
Target detection using ORB feature matching between the target
reference image and the robot camera frame.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np
from controller import Robot

from project_utils import CONFIG, ROOT, world_to_grid, grid_to_world


CROP_FRACTION = 0.60        #keep the upper part of the image where the poster appears
UPSCALE = 3                 #enlarge the small camera image before finding features
REF_SIZE = 256              #reference images are shrunk to a comparable scale
RATIO = 0.80                #lowe ratio test
RANSAC_THRESHOLD = 6.0      #pixels (in the upscaled image)
MIN_MATCHES_FOR_H = 6       #need clearly more than 4 points for a trustworthy homography
MIN_INLIERS = 10            #inliers needed to call something the target
MIN_MARGIN = 2.0            #target must beat the best OTHER image by this factor
FRAMES_PER_DECISION = 7     #decide from several frames, not one noisy frame

sift = cv2.SIFT_create(nfeatures=600, contrastThreshold=0.02)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))


# function to load target image
def load_target_image(target_name):
    target_path = ROOT / "textures" / f"target_{target_name}.png"
    image = cv2.imread(str(target_path))
    if image is None:
        raise FileNotFoundError(f"Cannot read {target_path}")
    return image


#find SIFT features in an image optionally enlarged first
def find_features(image, upscale=1):
    if upscale != 1:
        image = cv2.resize(image, None, fx=upscale, fy=upscale,
                           interpolation=cv2.INTER_CUBIC)
    #convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #boost local contrast so the washed out low-resolution poster keeps texture
    gray = clahe.apply(gray)
    #find keypoints and calculate their descriptors
    keypoints, descriptors = sift.detectAndCompute(gray, None)
    return keypoints, descriptors


#match SIFT features between the reference and camera image
def match_features(reference_descriptors, frame_descriptors):
    #SIFT descriptors are floating point vectors, so use L2 distance
    matcher = cv2.BFMatcher(cv2.NORM_L2)

    #find the two closest camera features for each reference feature
    knn_matches = matcher.knnMatch(reference_descriptors, frame_descriptors, k=2)

    #keep a match only when the best candidate is clearly better than the second best
    good_matches = []
    for pair in knn_matches:
        if len(pair) == 2:
            best, second_best = pair
            if best.distance < RATIO * second_best.distance:
                good_matches.append(best)

    #prevent multiple reference features from using the same camera feature
    unique_matches = {}
    for match in good_matches:
        camera_feature = match.trainIdx
        if (camera_feature not in unique_matches or
                match.distance < unique_matches[camera_feature].distance):
            unique_matches[camera_feature] = match

    return list(unique_matches.values())


#count the matches that agree on one geometric transform homography + RANSAC
def verify_matches(reference_keypoints, frame_keypoints, matches):
    if len(matches) < MIN_MATCHES_FOR_H:
        return 0

    reference_points = np.float32(
        [reference_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    frame_points = np.float32(
        [frame_keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    H, mask = cv2.findHomography(reference_points, frame_points,
                                 cv2.RANSAC, RANSAC_THRESHOLD)
    if H is None or mask is None:
        return 0

    #reject degenerate or mirrored transforms a real poster is never mirrored
    det = np.linalg.det(H[:2, :2])
    if det < 1e-6:
        return 0

    return int(mask.sum())


#reference features for the mission target, every other target, and the
#distractor images used only as negative examples
def build_references():
    references = {}
    for label in CONFIG["target_labels"]:
        image = load_target_image(label)
        image = cv2.resize(image, (REF_SIZE, REF_SIZE), interpolation=cv2.INTER_AREA)
        references[("target", label)] = find_features(image)
    return references


#crop the camera image to the region where a poster appears
def crop_frame(frame):
    height = frame.shape[0]
    return frame[0:int(height * CROP_FRACTION), :]


#score ONE camera frame against every reference: {key: geometric inliers}
def score_frame_all(frame, references):
    frame_keypoints, frame_descriptors = find_features(crop_frame(frame), UPSCALE)
    scores = {}
    for key, (ref_keypoints, ref_descriptors) in references.items():
        if (frame_descriptors is None or ref_descriptors is None or
                len(frame_keypoints) < MIN_MATCHES_FOR_H):
            scores[key] = 0
            continue
        matches = match_features(ref_descriptors, frame_descriptors)
        scores[key] = verify_matches(ref_keypoints, frame_keypoints, matches)
    return scores


#split a score dict into (target score, best other score, best other name)
def target_vs_others(scores, target_name):
    target_score = scores.get(("target", target_name), 0)
    others = {k: v for k, v in scores.items() if k != ("target", target_name)}
    other_key = max(others, key=others.get) if others else None
    other_score = others[other_key] if other_key else 0
    return target_score, other_score, other_key


#collects evidence over several frames taken at ONE station / viewpoint
class StationJudge:
    def __init__(self):
        self.target_scores = []
        self.other_scores = []

    def add(self, target_score, other_score):
        self.target_scores.append(target_score)
        self.other_scores.append(other_score)

    def ready(self):
        return len(self.target_scores) >= FRAMES_PER_DECISION

    def evidence(self):
        #the median means a single noisy frame cannot decide the outcome
        if not self.target_scores:
            return 0.0, 0.0
        return float(np.median(self.target_scores)), float(np.median(self.other_scores))

    def is_target(self):
        t, o = self.evidence()
        return t >= MIN_INLIERS and t >= MIN_MARGIN * max(o, 1.0)

    def reset(self):
        self.target_scores, self.other_scores = [], []

