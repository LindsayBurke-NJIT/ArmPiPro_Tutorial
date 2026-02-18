import numpy as np
from scipy.spatial import cKDTree

def polar_to_cartesian(angles_rad: np.ndarray, ranges_mm: np.ndarray) -> np.ndarray:
    """Convert polar (angle, range) to cartesian (x, y) in robot frame. x forward, y left."""
    r = np.asarray(ranges_mm, dtype=float) / 1000.0  #mm -> m
    x = r * np.cos(angles_rad)
    y = r * np.sin(angles_rad)
    return np.column_stack((x, y))

def apply_transform(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Apply pose (x, y, theta) to points (N,2). Returns world-frame points."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return (points @ R.T) + np.array([x, y])

def icp_2d(source: np.ndarray, target: np.ndarray, max_iter: int = 20, tol: float = 1e-4) -> tuple:
    src = np.asarray(source, dtype=float)
    if len(src) < 4 or len(target) < 4:
        return 0.0, 0.0, 0.0, src
    tree = cKDTree(target)
    x, y, theta = 0.0, 0.0, 0.0
    for _ in range(max_iter):
        transformed = apply_transform(src, x, y, theta)
        d, idx = tree.query(transformed, k=1)
        mask = d < 1.0
        if np.sum(mask) < 4:
            break
        src_m = transformed[mask]
        tgt_m = target[idx[mask]]
        src_c = src_m - src_m.mean(axis=0)
        tgt_c = tgt_m - tgt_m.mean(axis=0)
        
        #SVD for rotation
        H = src_c.T @ tgt_c
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        dtheta = np.arctan2(R[1, 0], R[0, 0])
        t = tgt_m.mean(axis=0) - (R @ src_m.mean(axis=0))
        dx, dy = t[0], t[1]
        x += dx
        y += dy
        theta += dtheta
        if abs(dx) < tol and abs(dy) < tol and abs(dtheta) < tol:
            break
    aligned = apply_transform(src, x, y, theta)
    return x, y, theta, aligned


class SLAM:
    """LiDAR SLAM: ICP pose estimation + log-odds occupancy grid."""

    def __init__(
        self,
        resolution: float = 0.05,
        width_m: float = 20.0,
        height_m: float = 20.0,
        log_odds_max: float = 10.0,
        log_odds_min: float = -10.0,
        prob_hit: float = 0.7,
        prob_miss: float = 0.4,
    ):
        self.resolution = resolution
        self.width_m, self.height_m = width_m, height_m
        self.origin = np.array([width_m / 2, height_m / 2])
        nw = int(width_m / resolution)
        nh = int(height_m / resolution)
        self.grid = np.zeros((nh, nw), dtype=float)  # log-odds
        self.log_odds_max = log_odds_max
        self.log_odds_min = log_odds_min
        self.lp_hit = np.log(prob_hit / (1 - prob_hit))
        self.lp_miss = np.log((1 - prob_miss) / prob_miss)
        self.pose = np.array([0.0, 0.0, 0.0])  # x, y, theta world
        self._prev_points = None  # previous scan in world frame for ICP
        self._map_points = []  # accumulated points for map (optional)

    def get_pose(self) -> tuple:
        """Return (x, y, theta) in meters and radians."""
        return float(self.pose[0]), float(self.pose[1]), float(self.pose[2])

    def world_to_cell(self, xy: np.ndarray) -> np.ndarray:
        """World (m) to grid indices."""
        ij = (xy + self.origin) / self.resolution
        return np.round(ij).astype(int)

    def in_bounds(self, ij: np.ndarray) -> np.ndarray:
        h, w = self.grid.shape
        return (ij[:, 0] >= 0) & (ij[:, 0] < w) & (ij[:, 1] >= 0) & (ij[:, 1] < h)

    def get_scan_points(self, laser, dmax: int = 10000) -> np.ndarray:
        """Get one scan from HokuyoLX and return cartesian points (N,2) in robot frame."""
        _, scan = laser.get_filtered_dist(dmax=dmax)
        if scan is None or len(scan) < 10:
            return np.empty((0, 2))
        angles = scan[:, 0]
        ranges = scan[:, 1]
        return polar_to_cartesian(angles, ranges)

    def update_map(self, points_world: np.ndarray):
        """Update occupancy grid with world-frame points (hits). Ray-cast misses from pose."""
        origin_ij = self.world_to_cell(self.pose[:2].reshape(1, 2))[0]
        hit_ij = self.world_to_cell(points_world)
        for i in range(len(points_world)):
            j, i_ = hit_ij[i, 0], hit_ij[i, 1]
            if 0 <= j < self.grid.shape[1] and 0 <= i_ < self.grid.shape[0]:
                self.grid[i_, j] = np.clip(
                    self.grid[i_, j] + self.lp_hit,
                    self.log_odds_min,
                    self.log_odds_max,
                )
        # Bresenham ray from origin to each hit to mark free
        for i in range(0, len(points_world), max(1, len(points_world) // 50)):
            j, i_ = hit_ij[i, 0], hit_ij[i, 1]
            for (jj, ii) in _bresenham(origin_ij[0], origin_ij[1], j, i_):
                if 0 <= jj < self.grid.shape[1] and 0 <= ii < self.grid.shape[0]:
                    self.grid[ii, jj] = np.clip(
                        self.grid[ii, jj] + self.lp_miss,
                        self.log_odds_min,
                        self.log_odds_max,
                    )

    def process_scan(self, points_robot: np.ndarray) -> bool:
        """
        Run ICP vs previous scan, update pose, update map. Returns True if successful.
        """
        if len(points_robot) < 10:
            return False
        if self._prev_points is None:
            self._prev_points = apply_transform(
                points_robot, self.pose[0], self.pose[1], self.pose[2]
            )
            self.update_map(self._prev_points)
            return True
        target = self._prev_points
        dx, dy, dtheta, aligned = icp_2d(points_robot, target)
        self.pose[0] += dx
        self.pose[1] += dy
        self.pose[2] += dtheta
        points_world = apply_transform(
            points_robot, self.pose[0], self.pose[1], self.pose[2]
        )
        self.update_map(points_world)
        self._prev_points = points_world
        return True

    def step(self, laser) -> bool:
        """Acquire one scan, process it, update map and pose. Returns True on success."""
        points = self.get_scan_points(laser)
        return self.process_scan(points)

    def get_map(self) -> np.ndarray:
        """Return occupancy grid (height, width). Values are log-odds; >0 occupied, <0 free."""
        return self.grid.copy()

    def get_map_prob(self) -> np.ndarray:
        """Return occupancy as probabilities in [0,1]. 0.5=unknown, >0.5 occupied."""
        p = 1.0 - 1.0 / (1.0 + np.exp(self.grid))
        return np.clip(p, 0.0, 1.0)


def _bresenham(x0: int, y0: int, x1: int, y1: int):
    """Yield grid cells along line from (x0,y0) to (x1,y1)."""
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        yield (x, y)
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
