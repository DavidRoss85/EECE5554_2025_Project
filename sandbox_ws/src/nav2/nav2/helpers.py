import heapq
import math
import numpy as np
from scipy.ndimage import binary_dilation

OPEN_CELL_THRESHOLD = 50


# --- Helper functions ---

def quaternion_to_yaw(q):
    # Convert quaternion to yaw (rotation about Z)
    try:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    except Exception as e:
        return 0

def heuristic(a, b):
    """Euclidean distance between two grid cells."""
    return math.hypot(a[0] - b[0], a[1] - b[1])

def open_neighbors(node, grid,threshold=50):
    """Generate valid neighboring cells (8-connected)."""
    neigboring_set =  [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
    i, j = node
    map_height, map_width = grid.shape
    map_min_height = 0; map_min_width = 0

    for di, dj in neigboring_set:
        ni, nj = i + di, j + dj
        if map_min_height <= ni < map_height\
        and map_min_width <= nj < map_width:
            if grid[ni, nj] < threshold:  # threshold for free space
                yield (ni, nj)

def find_a_star_path(start, goal,grid=None,threshold=OPEN_CELL_THRESHOLD,weight=1.0):
    """
    Compute an A* path on a 2D occupancy grid.
    Args:
        start: [i, j] grid coordinates (row, column)
        goal:  [i, j] grid coordinates (row, column)
    Returns:
        path: list of [i, j] grid coordinates from start to goal
    """
    if grid is None:
        return []


    # --- A* setup ---
    sx,sy = start; gx,gy = goal
    start = (sx,sy); goal = (gx,gy) #Ensure start/goal is immutable\
    
    open_set = []
    h=heuristic(start,goal) * weight
    heapq.heappush(open_set, (h, 0, start))
    came_from = {}
    g_score = {start: 0}

    
    # --- A* search loop ---
    while open_set:
        _, current_cost, current = heapq.heappop(open_set)

        # print(f"\ncurrent: {current}\ngoal: {goal}\n")
        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            results = [list(p) for p in path]  # list of [i, j]
            return results

        for neighbor in open_neighbors(current,grid,threshold):
            tentative_g = g_score[current] + heuristic(current, neighbor)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                h = weight * heuristic(neighbor, goal)
                f_score = tentative_g + h
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                came_from[neighbor] = current


def inflate_obstacles(occ_grid, robot_width, resolution, threshold=OPEN_CELL_THRESHOLD):
    """
    Inflates occupied cells in a binary occupancy grid to account for robot width.

    Args:
        occ_grid: 2D NumPy array (0=free, 100=occupied, -1/50=unknown)
        robot_width: robot diameter in meters
        resolution: map resolution (m per cell)
        threshold: occupancy threshold above which cells are considered occupied
    Returns:
        inflated_grid: new occupancy map (same shape)
    """
    # Convert map to boolean: True = obstacle
    obstacle_mask = occ_grid >= threshold

    # Compute inflation radius in cells
    inflation_radius = int(np.ceil((robot_width / 2.0) / resolution))

    # Build circular structuring element (kernel)
    y, x = np.ogrid[-inflation_radius:inflation_radius+1,
                    -inflation_radius:inflation_radius+1]
    kernel = x**2 + y**2 <= inflation_radius**2

    # Perform binary dilation
    inflated_mask = binary_dilation(obstacle_mask, structure=kernel)

    # Build new map: 100 where inflated, else original
    inflated_grid = occ_grid.copy()
    inflated_grid[inflated_mask] = 100

    return inflated_grid