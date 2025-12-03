import heapq
import math
import numpy as np
from scipy.ndimage import binary_dilation

USING_GAZEBO = False

OPEN_CELL_THRESHOLD = 50
A_STAR_HEAVY_WEIGHT = 5
A_STAR_LOOSE_THRESHOLD = 55


# --- Helper functions ---
#--------------------------------------------------------------------------------
def quaternion_to_yaw(q):
    """
    Convert a quaternion into yaw angle (in radians).
    Args:
        q: geometry_msgs.msg.Quaternion
    Returns:
        yaw angle in radians
    """
    # Convert quaternion to yaw (rotation about Z)
    try:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    except Exception as e:
        return 0
#--------------------------------------------------------------------------------
def transform_2d(tx, ty, theta, x, y):
    # Rotation
    # [x']   [cosθ  -sinθ tx]   [x]
    # [y'] = [sinθ   cosθ ty] * [y]
    # [z']   [ 0      0    1]   [1]
            
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    # Apply rotation + translation
    item_x = tx + cos_t * x - sin_t * y
    item_y = ty + sin_t * x + cos_t * y

    return item_x, item_y

#--------------------------------------------------------------------------------
def heuristic(a, b):
    """Euclidean distance between two grid cells."""
    return math.hypot(a[0] - b[0], a[1] - b[1])
#--------------------------------------------------------------------------------
def open_neighbors(node, grid,threshold=50):
    """Generate valid neighboring cells (8-connected)."""
    neigboring_set =  [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
    i, j = node
    map_height, map_width = grid.shape
    map_min_height = 0; map_min_width = 0

    for di, dj in neigboring_set:
        ni, nj = i + di, j + dj
        ni = min(max(map_min_height,nj),map_height-1)
        nj = min(max(map_min_width,nj),map_width-1)
        # ni = map_min_height if ni < map_min_height else map_height-1 if ni >= map_height else ni
        # nj = map_min_width if nj < map_min_width else map_width-1 if nj >= map_width else nj
        # if map_min_height <= ni < map_height\
        # and map_min_width <= nj < map_width:
        if grid[ni, nj] < threshold:  # threshold for free space
            yield (ni, nj)
#--------------------------------------------------------------------------------
def find_a_star_path(start, goal, grid=None, threshold=OPEN_CELL_THRESHOLD, weight=1.0, optimistic=True):
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
    si, sj = start
    gi, gj = goal
    start = (si, sj)

    if optimistic:
        weight = A_STAR_HEAVY_WEIGHT
        threshold = A_STAR_LOOSE_THRESHOLD
        height, width = grid.shape
        gi = min(gi, height - 1)   # clip row (i)
        gj = min(gj, width - 1)    # clip column (j)

    goal = (gi, gj)  # Ensure goal is immutable

    open_set = []
    h = heuristic(start, goal) * weight
    heapq.heappush(open_set, (h, 0, start))
    came_from = {}
    g_score = {start: 0}
    current = start
    # --- A* search loop ---
    while open_set:
        _, current_cost, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return [list(p) for p in path]

        for neighbor in open_neighbors(current, grid, threshold):
            tentative_g = g_score[current] + heuristic(current, neighbor)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                h = weight * heuristic(neighbor, goal)
                f_score = tentative_g + h
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                came_from[neighbor] = current
    
    # If goal not reached, try to path to the closest block reached:
    if optimistic and g_score:
        # Find explored node closest to goal
        best_node = min(g_score.keys(), key=lambda n: heuristic(n, goal))

        # Reconstruct path from best_node back to start
        path = [best_node]
        while best_node in came_from:
            best_node = came_from[best_node]
            path.append(best_node)
        path.reverse()
        return [list(p) for p in path]
    

#--------------------------------------------------------------------------------
def inflate_obstacles(occ_grid, inflation_factor, resolution, threshold=OPEN_CELL_THRESHOLD):
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
    inflation_radius = int(np.ceil((inflation_factor / 2.0) / resolution))

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

#--------------------------------------------------------------------------------
def inflate_obstacles2(occ_grid, inflation_factor, resolution, threshold=OPEN_CELL_THRESHOLD):
    """
    Inflates occupied cells by spreading their own value outward
    instead of using a generic constant.
    """

    inflated_grid = occ_grid.copy()

    # Compute radius in cells
    inflation_radius = int(np.ceil((inflation_factor / 2.0) / resolution))

    # Create circular structuring element
    y, x = np.ogrid[-inflation_radius:inflation_radius+1,
                    -inflation_radius:inflation_radius+1]
    kernel = (x**2 + y**2) <= inflation_radius**2

    # Iterate through each original obstacle cell
    obstacle_indices = np.argwhere(occ_grid >= threshold)

    for (i, j) in obstacle_indices:
        val = occ_grid[i, j]

        # Compute window bounds
        i_min = max(i - inflation_radius, 0)
        i_max = min(i + inflation_radius, occ_grid.shape[0] - 1)
        j_min = max(j - inflation_radius, 0)
        j_max = min(j + inflation_radius, occ_grid.shape[1] - 1)

        # Extract sub-kernel for border cases
        k_i_min = i_min - (i - inflation_radius)
        k_i_max = kernel.shape[0] - ((i + inflation_radius) - i_max)
        k_j_min = j_min - (j - inflation_radius)
        k_j_max = kernel.shape[1] - ((j + inflation_radius) - j_max)

        mask = kernel[k_i_min:k_i_max, k_j_min:k_j_max]

        # Inflate values: use max so overlapping areas preserve highest occupancy
        inflated_grid[i_min:i_max+1, j_min:j_max+1][mask] = np.maximum(
            inflated_grid[i_min:i_max+1, j_min:j_max+1][mask],
            val
        )

    return inflated_grid

#--------------------------------------------------------------------------------
import numpy as np

def inflate_gaussian_multiclass(
        occ_grid,
        inflation_radius_m,
        resolution,
        sigma=None,
        class_threshold=OPEN_CELL_THRESHOLD
    ):
    """
    Inflates a semantic occupancy grid using Gaussian spreading.

    Whole number = class ID (e.g., 101 = person)
    Decimal part = confidence (0–1)

    Rules:
      • Only cells whose CLASS_ID >= class_threshold will be inflated.
      • Same-class overlap → probabilistic OR (a + b - ab)
      • Different-class overlap → keep class with higher confidence
      • Gaussian contributes fractional confidence only
    """

    inflated = occ_grid.copy().astype(float)

    # Convert inflation radius from meters to cells
    r = int(np.ceil(inflation_radius_m / resolution))

    # Default sigma is radius/2
    if sigma is None:
        sigma = r / 2.0

    # Precompute Gaussian kernel
    y, x = np.ogrid[-r:r+1, -r:r+1]
    d2 = x**2 + y**2
    kernel = np.exp(-(d2) / (2 * sigma * sigma))

    # Find all cells with class >= threshold
    class_cells = np.argwhere(np.floor(occ_grid) >= class_threshold)

    for (ci, cj) in class_cells:

        base_val = occ_grid[ci, cj]
        base_class = int(base_val)

        # Gaussian window boundaries
        i_min = max(ci - r, 0)
        i_max = min(ci + r, occ_grid.shape[0] - 1)
        j_min = max(cj - r, 0)
        j_max = min(cj + r, occ_grid.shape[1] - 1)

        # Subsample the kernel for edges
        k_i_min = i_min - (ci - r)
        k_i_max = k_i_min + (i_max - i_min + 1)
        k_j_min = j_min - (cj - r)
        k_j_max = k_j_min + (j_max - j_min + 1)

        subkernel = kernel[k_i_min:k_i_max, k_j_min:k_j_max]

        # Apply Gaussian spreading
        for ii in range(i_min, i_max + 1):
            for jj in range(j_min, j_max + 1):

                new_fraction = subkernel[ii - i_min, jj - j_min]

                # If Gaussian influence becomes extremely small, skip it
                if new_fraction < 1e-6:
                    continue

                existing_val = inflated[ii, jj]
                existing_class = int(existing_val)
                existing_fraction = existing_val - existing_class

                # CASE A — Same class → probabilistic OR
                if existing_class == base_class:

                    combined = existing_fraction + new_fraction - (existing_fraction * new_fraction)
                    combined = min(combined, 0.9999)

                    inflated[ii, jj] = base_class + combined

                # CASE B — Different class → keep class with higher confidence
                else:
                    if new_fraction > existing_fraction:
                        inflated[ii, jj] = base_class + min(new_fraction, 0.9999)

    return inflated

#-------------------------------------------------------------------
YOLO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"
]
