import math
import time
import os
import numpy as np
from scipy.spatial import cKDTree
import logging
import matplotlib
import matplotlib.pyplot as plt
####################################################
logger = logging.getLogger(__name__)

def polar_to_cartesian(angles_rad: np.ndarray, ranges_mm: np.ndarray) -> np.ndarray:
    """Convert polar (angle, range) to cartesian (x, y) in robot frame. x forward, y left."""
    try:
        angles_rad = np.asarray(angles_rad, dtype=float)
        ranges_mm = np.asarray(ranges_mm, dtype=float)
        
        if angles_rad.size == 0 or ranges_mm.size == 0:
            return np.empty((0, 2))
        
        if angles_rad.shape != ranges_mm.shape:
            raise ValueError(f"Shape mismatch: angles {angles_rad.shape} vs ranges {ranges_mm.shape}")
        
        valid_mask = (ranges_mm > 0) & np.isfinite(ranges_mm) & np.isfinite(angles_rad)
        if not np.any(valid_mask):
            return np.empty((0, 2))
        
        angles_rad = angles_rad[valid_mask]
        ranges_mm = ranges_mm[valid_mask]
        
        r = ranges_mm / 1000.0  #mm -> m
        x = r*np.cos(angles_rad)
        y = r*np.sin(angles_rad)
        return np.column_stack((x, y))
    except Exception as e:
        logger.error(f"Error in polar_to_cartesian: {e}")
        raise RuntimeError(f"Failed to convert polar to cartesian: {e}") from e

def apply_transform(points: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    """Apply pose (x, y, theta) to points (N,2). Returns world-frame points."""
    try:
        points = np.asarray(points, dtype=float)
        if points.size == 0:
            return np.empty((0, 2))
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError(f"Points must be (N,2), got shape {points.shape}")
        
        x, y, theta = float(x), float(y), float(theta)
        
        if not all(np.isfinite([x, y, theta])):
            raise ValueError(f"Invalid transform parameters: x={x}, y={y}, theta={theta}")
        
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        result = (points @ R.T) + np.array([x, y])
        
        if not np.all(np.isfinite(result)):
            raise ValueError("Transform produced non-finite values")
        
        return result
    except Exception as e:
        logger.error(f"Error in apply_transform: {e}")
        raise RuntimeError(f"Failed to apply transform: {e}") from e

def icp_2d(source: np.ndarray, target: np.ndarray, max_iter: int = 20, tol: float = 1e-4) -> tuple:
    """ICP alignment. Returns (dx, dy, dtheta, aligned_points)."""
    try:
        src = np.asarray(source, dtype=float)
        target = np.asarray(target, dtype=float)
        
        if src.size == 0 or target.size == 0:
            logger.warning("Empty point clouds in ICP")
            return 0.0, 0.0, 0.0, src
        
        if src.ndim != 2 or src.shape[1] != 2:
            raise ValueError(f"Source must be (N,2), got {src.shape}")
        if target.ndim != 2 or target.shape[1] != 2:
            raise ValueError(f"Target must be (N,2), got {target.shape}")
        
        if len(src) < 4 or len(target) < 4:
            logger.warning(f"Insufficient points for ICP: src={len(src)}, target={len(target)}")
            return 0.0, 0.0, 0.0, src
        
        #get rid of invalid points
        src_valid = np.all(np.isfinite(src), axis=1)
        tgt_valid = np.all(np.isfinite(target), axis=1)
        src = src[src_valid]
        target = target[tgt_valid]
        
        if len(src) < 4 or len(target) < 4:
            logger.warning("Too few valid points after filtering")
            return 0.0, 0.0, 0.0, src
        
        try:
            tree = cKDTree(target)
        except Exception as e:
            raise RuntimeError(f"Failed to build KDTree: {e}") from e
        
        x, y, theta = 0.0, 0.0, 0.0
        
        for iter_num in range(max_iter):
            try:
                transformed = apply_transform(src, x, y, theta)
                d, idx = tree.query(transformed, k=1)
                
                #filter outliers (1m threshold)
                mask = d < 1.0
                if np.sum(mask) < 4:
                    logger.debug(f"ICP: insufficient inliers at iteration {iter_num}")
                    break
                
                src_m = transformed[mask]
                tgt_m = target[idx[mask]]
                
                #center points
                src_c = src_m - src_m.mean(axis=0)
                tgt_c = tgt_m - tgt_m.mean(axis=0)
                
                if np.linalg.norm(src_c) < 1e-6 or np.linalg.norm(tgt_c) < 1e-6:
                    logger.debug(f"ICP: degenerate point clouds at iteration {iter_num}")
                    break
                
                #SVD for rotation
                H = src_c.T @ tgt_c
                if not np.all(np.isfinite(H)):
                    logger.warning(f"ICP: non-finite values in H matrix at iteration {iter_num}")
                    break
                
                try:
                    U, _, Vt = np.linalg.svd(H)
                except np.linalg.LinAlgError as e:
                    logger.warning(f"ICP: SVD failed at iteration {iter_num}: {e}")
                    break
                
                R = Vt.T @ U.T
                if np.linalg.det(R) < 0:
                    Vt[-1] *= -1
                    R = Vt.T @ U.T
                
                dtheta = np.arctan2(R[1, 0], R[0, 0])
                t = tgt_m.mean(axis=0) - (R @ src_m.mean(axis=0))
                dx, dy = float(t[0]), float(t[1])
                
                if not all(np.isfinite([dx, dy, dtheta])):
                    logger.warning(f"ICP: non-finite transform at iteration {iter_num}")
                    break
                
                x += dx
                y += dy
                theta += dtheta
                
                #normalize theta to [-pi, pi]
                theta = np.arctan2(np.sin(theta), np.cos(theta))
                
                if abs(dx) < tol and abs(dy) < tol and abs(dtheta) < tol:
                    logger.debug(f"ICP converged at iteration {iter_num}")
                    break
            except Exception as e:
                logger.error(f"ICP iteration {iter_num} failed: {e}")
                break
        
        try:
            aligned = apply_transform(src, x, y, theta)
        except Exception as e:
            logger.error(f"Failed to apply final ICP transform: {e}")
            aligned = src
        
        return x, y, theta, aligned
    except Exception as e:
        logger.error(f"Error in icp_2d: {e}")
        raise RuntimeError(f"ICP alignment failed: {e}") from e

def _bresenham(x0: int, y0: int, x1: int, y1: int):
    """Yield grid cells along line from (x0,y0) to (x1,y1)."""
    try:
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
        
        max_iter = 10000
        iter_count = 0
        
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        
        while iter_count < max_iter:
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
            iter_count += 1
        
        if iter_count >= max_iter:
            logger.warning(f"Bresenham exceeded max iterations: ({x0},{y0}) -> ({x1},{y1})")
    except Exception as e:
        logger.error(f"Error in Bresenham line: {e}")
        yield (x0, y0)

class SLAM:
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
        try:
            if resolution <= 0 or not np.isfinite(resolution):
                raise ValueError(f"Resolution must be positive and finite, got {resolution}")
            if width_m <= 0 or height_m <= 0:
                raise ValueError(f"Map dimensions must be positive: width={width_m}, height={height_m}")
            if not all(np.isfinite([width_m, height_m, log_odds_max, log_odds_min])):
                raise ValueError("All parameters must be finite")
            if not (0 < prob_hit < 1):
                raise ValueError(f"prob_hit must be in (0,1), got {prob_hit}")
            if not (0 < prob_miss < 1):
                raise ValueError(f"prob_miss must be in (0,1), got {prob_miss}")
            if log_odds_min >= log_odds_max:
                raise ValueError(f"log_odds_min ({log_odds_min}) must be < log_odds_max ({log_odds_max})")
            
            self.resolution = float(resolution)
            self.width_m = float(width_m)
            self.height_m = float(height_m)
            self.origin = np.array([width_m / 2, height_m / 2])
            
            nw = int(width_m / resolution)
            nh = int(height_m / resolution)
            
            if nw <= 0 or nh <= 0:
                raise ValueError(f"Invalid grid size: {nw}x{nh}")
            if nw > 10000 or nh > 10000:
                raise ValueError(f"Grid too large: {nw}x{nh}. Consider larger resolution.")
            
            self.grid = np.zeros((nh, nw), dtype=float)
            self.log_odds_max = float(log_odds_max)
            self.log_odds_min = float(log_odds_min)
            
            try:
                self.lp_hit = np.log(prob_hit / (1 - prob_hit))
                self.lp_miss = np.log((1 - prob_miss) / prob_miss)
            except (ValueError, ZeroDivisionError) as e:
                raise ValueError(f"Invalid probability values: prob_hit={prob_hit}, prob_miss={prob_miss}") from e
            
            if not np.isfinite(self.lp_hit) or not np.isfinite(self.lp_miss):
                raise ValueError("Computed log-probabilities are not finite")
            
            self.pose = np.array([0.0, 0.0, 0.0])  #x, y, theta world
            self._prev_points = None  #prev scan in world frame for ICP
            self._map_points = []  #accumulated points for map
            
            logger.info(f"SLAM initialized: grid={nh}x{nw}, resolution={resolution}m")
        except Exception as e:
            logger.error(f"Failed to initialize SLAM: {e}")
            raise RuntimeError(f"SLAM initialization failed: {e}") from e

    def get_pose(self) -> tuple:
        """Return (x, y, theta) in meters and radians."""
        try:
            if not np.all(np.isfinite(self.pose)):
                logger.warning("Pose contains non-finite values, returning zeros")
                return 0.0, 0.0, 0.0
            return float(self.pose[0]), float(self.pose[1]), float(self.pose[2])
        except Exception as e:
            logger.error(f"Error getting pose: {e}")
            return 0.0, 0.0, 0.0

    def world_to_cell(self, xy: np.ndarray) -> np.ndarray:
        """World (m) to grid indices."""
        try:
            xy = np.asarray(xy, dtype=float)
            if xy.size == 0:
                return np.empty((0, 2), dtype=int)
            if xy.ndim == 1:
                xy = xy.reshape(1, -1)
            if xy.shape[1] != 2:
                raise ValueError(f"xy must be (N,2), got {xy.shape}")
            
            if not np.all(np.isfinite(xy)):
                logger.warning("Non-finite values in world_to_cell input")
                xy = np.nan_to_num(xy, nan=0.0, posinf=0.0, neginf=0.0)
            
            ij = (xy + self.origin) / self.resolution
            return np.round(ij).astype(int)
        except Exception as e:
            logger.error(f"Error in world_to_cell: {e}")
            raise RuntimeError(f"Failed to convert world to cell: {e}") from e

    def in_bounds(self, ij: np.ndarray) -> np.ndarray:
        """Check if grid indices are within bounds."""
        try:
            ij = np.asarray(ij, dtype=int)
            if ij.size == 0:
                return np.array([], dtype=bool)
            if ij.ndim == 1:
                ij = ij.reshape(1, -1)
            if ij.shape[1] != 2:
                raise ValueError(f"ij must be (N,2), got {ij.shape}")
            
            h, w = self.grid.shape
            return (ij[:, 0] >= 0) & (ij[:, 0] < w) & (ij[:, 1] >= 0) & (ij[:, 1] < h)
        except Exception as e:
            logger.error(f"Error in in_bounds: {e}")
            return np.array([False] * len(ij) if ij.size > 0 else [], dtype=bool)

    def get_scan_points(self, laser, dmax: int = 10000) -> np.ndarray:
        """Get one scan from HokuyoLX and return cartesian points (N,2) in robot frame."""
        try:
            if laser is None:
                raise ValueError("Laser object is None")
            
            if not (0 < dmax <= 30000):
                logger.warning(f"Invalid dmax={dmax}, using 10000")
                dmax = 10000
            
            try:
                _, scan = laser.get_filtered_dist(dmax=dmax)
            except AttributeError:
                raise RuntimeError("Laser object does not have get_filtered_dist method")
            except Exception as e:
                raise RuntimeError(f"Failed to get scan from laser: {e}") from e
            
            if scan is None:
                logger.warning("Laser returned None scan")
                return np.empty((0, 2))
            
            scan = np.asarray(scan)
            if scan.size == 0:
                logger.warning("Empty scan received")
                return np.empty((0, 2))
            
            if scan.ndim != 2 or scan.shape[1] < 2:
                raise RuntimeError(f"Invalid scan shape: {scan.shape}, expected (N,2)")
            
            if len(scan) < 10:
                logger.debug(f"Scan has only {len(scan)} points (minimum 10 recommended)")
            
            angles = scan[:, 0]
            ranges = scan[:, 1]
            
            return polar_to_cartesian(angles, ranges)
        except Exception as e:
            logger.error(f"Error in get_scan_points: {e}")
            raise RuntimeError(f"Failed to get scan points: {e}") from e

    def update_map(self, points_world: np.ndarray):
        """Update occupancy grid with world-frame points (hits). Ray-cast misses from pose."""
        try:
            points_world = np.asarray(points_world, dtype=float)
            if points_world.size == 0:
                logger.debug("Empty points_world in update_map")
                return
            
            if points_world.ndim != 2 or points_world.shape[1] != 2:
                raise ValueError(f"points_world must be (N,2), got {points_world.shape}")
            
            if not np.all(np.isfinite(points_world)):
                logger.warning("Non-finite values in points_world, filtering")
                valid = np.all(np.isfinite(points_world), axis=1)
                points_world = points_world[valid]
                if len(points_world) == 0:
                    return
            
            try:
                origin_ij = self.world_to_cell(self.pose[:2].reshape(1, 2))[0]
                hit_ij = self.world_to_cell(points_world)
            except Exception as e:
                raise RuntimeError(f"Failed to convert to grid coordinates: {e}") from e
            
            h, w = self.grid.shape
            for i in range(len(points_world)):
                try:
                    j, i_ = int(hit_ij[i, 0]), int(hit_ij[i, 1])
                    if 0 <= j < w and 0 <= i_ < h:
                        self.grid[i_, j] = np.clip(
                            self.grid[i_, j] + self.lp_hit,
                            self.log_odds_min,
                            self.log_odds_max,
                        )
                except (IndexError, ValueError) as e:
                    logger.debug(f"Skipping invalid hit index {i}: {e}")
                    continue
            
            #Ray-cast free cells
            step = max(1, len(points_world) // 50)
            for i in range(0, len(points_world), step):
                try:
                    j, i_ = int(hit_ij[i, 0]), int(hit_ij[i, 1])
                    origin_j, origin_i = int(origin_ij[0]), int(origin_ij[1])
                    
                    #skip if origin or hit is out of bounds
                    if not (0 <= origin_j < w and 0 <= origin_i < h):
                        continue
                    if not (0 <= j < w and 0 <= i_ < h):
                        continue
                    
                    for (jj, ii) in _bresenham(origin_j, origin_i, j, i_):
                        if 0 <= jj < w and 0 <= ii < h:
                            self.grid[ii, jj] = np.clip(
                                self.grid[ii, jj] + self.lp_miss,
                                self.log_odds_min,
                                self.log_odds_max,
                            )
                except (ValueError, IndexError) as e:
                    logger.debug(f"Skipping ray-cast for point {i}: {e}")
                    continue
        except Exception as e:
            logger.error(f"Error in update_map: {e}")
            raise RuntimeError(f"Failed to update map: {e}") from e

    def process_scan(self, points_robot: np.ndarray) -> bool:
        """
        Run ICP vs previous scan, update pose, update map. Returns True if successful.
        """
        try:
            points_robot = np.asarray(points_robot, dtype=float)
            
            if points_robot.size == 0:
                logger.warning("Empty points_robot in process_scan")
                return False
            
            if points_robot.ndim != 2 or points_robot.shape[1] != 2:
                logger.error(f"Invalid points_robot shape: {points_robot.shape}")
                return False
            
            if len(points_robot) < 10:
                logger.debug(f"Insufficient points for processing: {len(points_robot)}")
                return False
            
            #init with first scan
            if self._prev_points is None:
                try:
                    self._prev_points = apply_transform(
                        points_robot, self.pose[0], self.pose[1], self.pose[2]
                    )
                    self.update_map(self._prev_points)
                    logger.info("Initialized SLAM with first scan")
                    return True
                except Exception as e:
                    logger.error(f"Failed to initialize with first scan: {e}")
                    return False
            
            try:
                target = self._prev_points
                if len(target) < 10:
                    logger.warning("Previous scan has insufficient points, skipping ICP")
                    return False
                
                dx, dy, dtheta, aligned = icp_2d(points_robot, target)
                
                if not all(np.isfinite([dx, dy, dtheta])):
                    logger.warning("ICP returned non-finite transform, skipping update")
                    return False
                
                if abs(dx) > 5.0 or abs(dy) > 5.0:
                    logger.warning(f"Large ICP transform detected: dx={dx:.2f}, dy={dy:.2f}, skipping")
                    return False
                
                #update pose
                self.pose[0] += dx
                self.pose[1] += dy
                self.pose[2] += dtheta
                
                self.pose[2] = np.arctan2(np.sin(self.pose[2]), np.cos(self.pose[2]))
                
                #transform to world frame and update map
                points_world = apply_transform(
                    points_robot, self.pose[0], self.pose[1], self.pose[2]
                )
                
                self.update_map(points_world)
                self._prev_points = points_world
                
                return True
            except RuntimeError as e:
                logger.error(f"ICP failed: {e}")
                return False
            except Exception as e:
                logger.error(f"Error during scan processing: {e}")
                return False
        except Exception as e:
            logger.error(f"Error in process_scan: {e}")
            return False

    def step(self, laser) -> bool:
        """Acquire one scan, process it, update map and pose. Returns True on success."""
        try:
            if laser is None:
                logger.error("Laser is None in step()")
                return False
            
            try:
                points = self.get_scan_points(laser)
            except RuntimeError as e:
                logger.error(f"Failed to get scan: {e}")
                return False
            
            return self.process_scan(points)
        except Exception as e:
            logger.error(f"Error in step: {e}")
            return False

    def get_map(self) -> np.ndarray:
        """Return occupancy grid (height, width). Values are log-odds; >0 occupied, <0 free."""
        try:
            return self.grid.copy()
        except Exception as e:
            logger.error(f"Error getting map: {e}")
            return np.zeros_like(self.grid)

    def get_map_prob(self) -> np.ndarray:
        """Return occupancy as probabilities in [0,1]. 0.5=unknown, >0.5 occupied."""
        try:
            grid_clipped = np.clip(self.grid, -50, 50)
            p = 1.0 - 1.0 / (1.0 + np.exp(grid_clipped))
            result = np.clip(p, 0.0, 1.0)
            
            if not np.all(np.isfinite(result)):
                logger.warning("Non-finite values in probability map, clipping")
                result = np.nan_to_num(result, nan=0.5, posinf=1.0, neginf=0.0)
            
            return result
        except Exception as e:
            logger.error(f"Error getting probability map: {e}")
            return np.full_like(self.grid, 0.5, dtype=float)

    def save_map_visualization(
        self,
        out_dir: str = "maps",
        filename_prefix: str = "lidar_map",
        map_prob: np.ndarray | None = None,
        pose: tuple[float, float, float] | None = None,
        save_npy: bool = True,
    ) -> str:
        """
        Save the occupancy probability map as a PNG in the current output directory.
        """
        try:
            if map_prob is None:
                map_prob = self.get_map_prob()
            if pose is None:
                pose = self.get_pose()

            map_prob = np.asarray(map_prob, dtype=float)
            if map_prob.ndim != 2:
                raise ValueError(f"map_prob must be 2D (H,W), got shape {map_prob.shape}")

            os.makedirs(out_dir, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            base = f"{filename_prefix}_{ts}"
            png_path = os.path.join(out_dir, f"{base}.png")

            if save_npy:
                npy_path = os.path.join(out_dir, f"{base}.npy")
                np.save(npy_path, map_prob)

            matplotlib.use("Agg", force=True)

            #occupied (p~1)->dark; free (p~0)->light; unknown around mid-gray
            img = 1.0 - np.clip(map_prob, 0.0, 1.0)

            fig = plt.figure(figsize=(8, 8), dpi=150)
            ax = fig.add_subplot(111)
            ax.imshow(img, cmap="gray", origin="lower", interpolation="nearest")
            ax.set_title(base)
            ax.set_xlabel("grid x")
            ax.set_ylabel("grid y")

            try:
                x, y, theta = pose
                ij = self.world_to_cell(np.array([[x, y]], dtype=float))[0]
                j, i = int(ij[0]), int(ij[1])
                ax.plot([j], [i], "r.", markersize=6)

                arrow_len_cells = 10.0
                dx = arrow_len_cells * float(np.cos(theta))
                dy = arrow_len_cells * float(np.sin(theta))
                ax.arrow(
                    j,
                    i,
                    dx,
                    dy,
                    color="red",
                    width=0.5,
                    head_width=3.0,
                    length_includes_head=True,
                )
            except Exception:
                pass

            fig.tight_layout()
            fig.savefig(png_path)
            plt.close(fig)

            return png_path
        except Exception as e:
            logger.error(f"Failed to save map visualization: {e}")
            raise

    def get_obstacle_distances(self, laser, safe_distance_mm=500):
        """
        Get obstacle distances in different directions from lidar scan.
        Uses same coordinate system as SLAM: x forward, y left, angle 0 = forward.
        Returns a dictionary with distances in front, left, right, and back.
        safe_distance_mm: minimum safe distance in millimeters
        """
        try:
            timestamp, scan = laser.get_filtered_dist(dmax=10000)
            if scan is None or scan.size == 0:
                return None

            if scan.ndim != 2 or scan.shape[1] < 2:
                return None

            angles = scan[:, 0]  #in radians (0 = forward, pi/2 = left)
            ranges = scan[:, 1]  #in millimeters

            angles = np.arctan2(np.sin(angles), np.cos(angles))

            # Define sectors in robot frame (x forward, y left):
            # Front: -π/4 to π/4 (45 degrees around forward)
            # Left: π/4 to 3π/4 (90 degrees to the left)
            # Back: 3π/4 to -3π/4 (45 degrees behind)
            # Right: -3π/4 to -π/4 (90 degrees to the right)

            front_mask = (angles >= -np.pi/4) & (angles <= np.pi/4)
            left_mask = (angles >= np.pi/4) & (angles <= 3*np.pi/4)
            back_mask = (angles >= 3*np.pi/4) | (angles <= -3*np.pi/4)
            right_mask = (angles >= -3*np.pi/4) & (angles <= -np.pi/4)

            front_dist = np.min(ranges[front_mask]) if np.any(front_mask) else 10000
            left_dist = np.min(ranges[left_mask]) if np.any(left_mask) else 10000
            back_dist = np.min(ranges[back_mask]) if np.any(back_mask) else 10000
            right_dist = np.min(ranges[right_mask]) if np.any(right_mask) else 10000

            return {
                'front': front_dist,
                'right': right_dist,
                'left': left_dist,
                'back': back_dist,
                'min_distance': np.min(ranges),
                'has_obstacle_front': front_dist < safe_distance_mm,
                'has_obstacle_right': right_dist < safe_distance_mm,
                'has_obstacle_left': left_dist < safe_distance_mm,
                'has_obstacle_back': back_dist < safe_distance_mm
            }
        except Exception as e:
            logger.error(f"Error getting obstacle distances: {e}")
            return None

    def find_safe_direction(self, obstacle_info, preferred_angle_deg=0):
        """
        Find a safe direction to move based on obstacle information.
        preferred_angle_deg: preferred direction in degrees (0=forward, 90=left, -90=right, 180=back)
        Returns: (forward, strafe, rotation) values for drive_xy
        """
        if obstacle_info is None:
            return (30, 0, 0) #go forward slowly

        safe_distance = 500  #mm

        preferred_angle_deg = ((preferred_angle_deg + 180) % 360) - 180

        preferred_rad = math.radians(preferred_angle_deg)
        preferred_forward = math.cos(preferred_rad)
        preferred_strafe = -math.sin(preferred_rad)

        if abs(preferred_angle_deg) <= 45:  #forward sector
            if not obstacle_info['has_obstacle_front']:
                speed = 50
                return (int(preferred_forward * speed), int(preferred_strafe * speed), 0)
        elif preferred_angle_deg > 45 and preferred_angle_deg <= 135:  #left sector
            if not obstacle_info['has_obstacle_left']:
                speed = 50
                return (int(preferred_forward * speed), int(preferred_strafe * speed), 0)
        elif preferred_angle_deg < -45 and preferred_angle_deg >= -135:  #right sector
            if not obstacle_info['has_obstacle_right']:
                speed = 50
                return (int(preferred_forward * speed), int(preferred_strafe * speed), 0)
        else:  #back sector
            if not obstacle_info['has_obstacle_back']:
                speed = 30
                return (int(preferred_forward * speed), int(preferred_strafe * speed), 0)

        #if preferred direction is blocked, find alternative
        if not obstacle_info['has_obstacle_front']:
            return (40, 0, 0)
        elif not obstacle_info['has_obstacle_right']:
            return (0, 40, 0)
        elif not obstacle_info['has_obstacle_left']:
            return (0, -40, 0)
        elif not obstacle_info['has_obstacle_back']:
            return (-30, 0, 0)
        else:
            #if all directions blocked, rotate
            return (0, 0, 30)

    def explore_waypoints(self, chassis, laser, waypoints=None, max_iterations=1000):
        """
        Explore environment by moving to different waypoints with obstacle avoidance.
        waypoints: list of (x, y) tuples in meters (relative to start)
        """
        if waypoints is None:
            #Default exploration pattern (moves in square pattern)
            waypoints = [
                (1.0, 0.0),   # Forward 1m
                (1.0, 1.0),   # Forward and right
                (0.0, 1.0),   # Right
                (0.0, 0.0),   # Back to start
                (-1.0, 0.0),  # Backward
                (-1.0, -1.0), # Backward and left
                (0.0, -1.0),  # Left
                (0.0, 0.0),   # Back to start
            ]

        current_waypoint_idx = 0
        waypoint_reached_threshold = 0.3  #meters
        scan_interval = 5  #num of iterations between Lidar scan updates
        movement_duration = 0.2  #secs per movement step

        logger.info(f"Starting exploration with {len(waypoints)} waypoints")

        iteration = 0
        success_count = 0

        while iteration < max_iterations:
            try:
                x, y, theta = self.get_pose()
                obstacle_info = self.get_obstacle_distances(laser, safe_distance_mm=500)

                #check if at current waypoint
                if current_waypoint_idx < len(waypoints):
                    target_x, target_y = waypoints[current_waypoint_idx]
                    dx = target_x - x
                    dy = target_y - y
                    distance_to_waypoint = math.sqrt(dx*dx + dy*dy)

                    if distance_to_waypoint < waypoint_reached_threshold:
                        logger.info(f"Reached waypoint {current_waypoint_idx}: ({target_x:.2f}, {target_y:.2f})")
                        current_waypoint_idx += 1
                        if current_waypoint_idx >= len(waypoints):
                            logger.info("All waypoints reached! Continuing exploration...")
                            current_waypoint_idx = 0
                        time.sleep(0.5)
                        continue

                    world_angle = math.atan2(dy, dx)
                    robot_angle_rad = world_angle - theta
                    robot_angle_rad = math.atan2(math.sin(robot_angle_rad), math.cos(robot_angle_rad))
                    robot_angle = math.degrees(robot_angle_rad)

                    angle_error_deg = robot_angle
                    if abs(angle_error_deg) > 15:
                        rotation_adjustment = max(-30, min(30, angle_error_deg * 0.5))
                    else:
                        rotation_adjustment = 0
                else:
                    robot_angle = 0
                    rotation_adjustment = 0

                forward, strafe, rotation = self.find_safe_direction(obstacle_info, preferred_angle_deg=robot_angle)
                rotation += rotation_adjustment #realign to reach next waypoint
                rotation = max(-50, min(50, rotation))

                if obstacle_info and obstacle_info['min_distance'] < 300:
                    logger.warning(f"Obstacle too close ({obstacle_info['min_distance']:.0f}mm), rotating away...")
                    forward = 0
                    strafe = 0
                    if obstacle_info['right'] > obstacle_info['left']:
                        rotation = 40  #rotate right
                    else:
                        rotation = -40  #rotate left

                chassis.drive_xy(forward=forward, strafe=strafe, rotation=rotation)
                time.sleep(movement_duration)
                chassis.stop_motors()

                #Update SLAM
                if iteration % scan_interval == 0:
                    if self.step(laser):
                        success_count += 1
                        if iteration % 50 == 0:
                            logger.info(f"Iteration {iteration}: pose x={x:.2f} y={y:.2f} theta={math.degrees(theta):.1f}°")
                            if obstacle_info:
                                logger.info(
                                    f"  Obstacles - Front: {obstacle_info['front']:.0f}mm, "
                                    f"Right: {obstacle_info['right']:.0f}mm, "
                                    f"Left: {obstacle_info['left']:.0f}mm"
                                )
                iteration += 1
                time.sleep(0.05)

            except KeyboardInterrupt:
                logger.info("Exploration interrupted by user")
                break
            except Exception as e:
                logger.error(f"Error during exploration iteration {iteration}: {e}")
                time.sleep(0.1)
                continue

        logger.info(f"Exploration completed: {success_count} SLAM updates successful")
        return success_count