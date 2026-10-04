"""
Path smoothing, cuts the A* staircase down to a few straight lines by
skipping ahead to the furthest cell reachable in a straight clear line.
"""

import math

from project_utils import RES, grid_to_world

#Closest a line may pass an obstacle, e-puck radius plus margin
CLEARANCE_M = 0.045
#Gap between points checked along a line, in cells
SAMPLE_STEP = 0.2


#True if a point (in cell units) is too close to an obstacle
def too_close(grid, r, c):
    clearance = CLEARANCE_M / RES
    for rr in range(int(r) - 1, int(r) + 3):
        for cc in range(int(c) - 1, int(c) + 3):
            if not (0 <= rr < len(grid) and 0 <= cc < len(grid[0])) or grid[rr][cc] == 0:
                continue
            #Distance to the edge of this obstacle cell
            dr = max(abs(r - rr) - 0.5, 0.0)
            dc = max(abs(c - cc) - 0.5, 0.0)
            if math.hypot(dr, dc) < clearance:
                return True
    return False


#True if the line between two cells stays clear of obstacles
def line_of_sight(grid, a, b):
    (r0, c0), (r1, c1) = a, b
    steps = max(1, math.ceil(math.hypot(r1 - r0, c1 - c0) / SAMPLE_STEP))
    for i in range(steps + 1):
        t = i / steps
        if too_close(grid, r0 + (r1 - r0) * t, c0 + (c1 - c0) * t):
            return False
    return True


#Drop points that sit on a straight line
def merge_straight(points):
    merged = points[:2]
    for p in points[2:]:
        (r0, c0), (r1, c1) = merged[-2], merged[-1]
        if (r1 - r0) * (p[1] - c1) == (c1 - c0) * (p[0] - r1):
            merged[-1] = p
        else:
            merged.append(p)
    return merged


#Jump to the furthest path cell still in a clear straight line
def smooth_path(grid, path):
    if not path or len(path) < 2:
        return path

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_of_sight(grid, path[i], path[j]):
            j -= 1
        smoothed.append(path[j])
        i = j
    return merge_straight(smoothed)


#Smoothed path as world waypoints skipping the start cell
def smooth_waypoints(grid, path):
    return [grid_to_world(*cell) for cell in smooth_path(grid, path)[1:]]


#Quick test to see if the smoothing actually works in a vacuum
if __name__ == "__main__":
    open_grid = [[0] * 10 for _ in range(10)]
    staircase = [(0, 0), (1, 0), (1, 1), (2, 1), (2, 2), (3, 2), (3, 3), (4, 3), (4, 4)]
    smoothed = smooth_path(open_grid, staircase)
    print("open grid:", smoothed)
    assert smoothed == [(0, 0), (4, 4)], "open staircase should become one straight line"

    #Wall with a gap on the right, lines must not cut through it
    wall_grid = [[0] * 10 for _ in range(10)]
    for c in range(8):
        wall_grid[5][c] = 1
    around = [(r, 0) for r in range(0, 3)] + [(2, c) for c in range(1, 10)] \
        + [(r, 9) for r in range(3, 8)] + [(8, c) for c in range(8, -1, -1)]
    smoothed = smooth_path(wall_grid, around)
    print("wall grid:", smoothed)
    for a, b in zip(smoothed, smoothed[1:]):
        assert line_of_sight(wall_grid, a, b), f"segment {a} -> {b} cuts the wall"
    assert len(smoothed) < len(around), "should still remove some cells"

    #Straight run along a wall should collapse to its two ends
    run = [(0, c) for c in range(6)]
    assert merge_straight(run) == [(0, 0), (0, 5)], "straight run should merge"

    print("smoothing helper dummy test passed")
