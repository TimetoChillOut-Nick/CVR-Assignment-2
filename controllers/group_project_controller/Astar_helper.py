import heapq
"""
Using A* to find the shortest distance to the closest view station.
To keep it simple we used manhatten Distance as it seemed logical to use with the grid layout
only being 4 directions (U-D-L-R)
"""

def astar(grid, start, goal):
    rows, cols = grid.shape

    #Check if the cell (row, cols)is in the grid
    def in_bounds(cell):
        r, c = cell
        return 0 <= r < rows and 0 <= c < cols

    #true if a free space (0=passable, 1=object)
    def passable(cell):
        r, c = cell
        return grid[r, c] == 0

    #Check if either is possible, and check if bot is already at destination
    if not in_bounds(goal) or not passable(goal):
        return None
    if start == goal:
        return [start]


    #Manhattan distance is the sum of row/col differences with no diagonals
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_heap = [(heuristic(start, goal), start)]  #cells left to explore cheapest first
    came_from = {}  #Origin Cell
    g_score = {start: 0}  #cheapest cost from start cell

    while open_heap:
        _, current = heapq.heappop(open_heap)  #grab the cheapest unexplored cell
        if current == goal:
            #Walk back through to came_from to build the path then reverse it
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        r, c = current
        for neighbor in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):  #up, down, left, right
            if not in_bounds(neighbor) or not passable(neighbor):
                continue
            tentative_g = g_score[current] + 1  #cost to reach neighbor
            if tentative_g < g_score.get(neighbor, float("inf")):
                #found a cheaper way to reach neighbor and store it
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return None  #ran out of cells to explore, goal is unreachable



#Quick test to see if the function actually works in a vacuum
if __name__ == "__main__":
    import numpy as np

    test_grid = np.array([
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0],
    ])

    path = astar(test_grid, (0, 0), (4, 4))
    print("path:", path)

    assert path is not None, "no path"
    assert path[0] == (0, 0) and path[-1] == (4, 4), "path should start/end at start/goal"
    assert all(test_grid[r, c] == 0 for r, c in path), "path ignores an obstacle"

    print("astar dummy test passed")