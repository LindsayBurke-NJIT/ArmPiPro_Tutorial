class Config:
    """Runtime settings for SLAM exploration and map export. Edit defaults below."""

    exploration_mode: str = "waypoints"
    max_iterations: int = 5000
    free_heading_reseed_interval: int = 25
    waypoints: tuple[tuple[float, float], ...] = (
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (0.0, 0.0),
        (-1.0, 0.0),
        (-1.0, -1.0),
        (0.0, -1.0),
        (0.0, 0.0),
    ) #exploration_mode "free" ignores waypoints at runtime
    slam_resolution: float = 0.05
    map_out_dir: str = "maps"
    map_filename_prefix: str = "lidar_map"
    save_map_npy: bool = True

    #settings for lidar obstacle avoidance
    lidar_mask_angle_intervals_deg: tuple[tuple[float, float], ...] = () #sets the angle intervals to mask for obstacle avoidance (e.g. range where the arm/chassis of robot is to avoid sensing itself)
    lidar_forward_cone_half_width_deg: float = 40.0 #sets the width of the forward cone for obstacle avoidance
    lidar_forward_clearance_percentile: float = 10.0 #sets the percentile of the forward cone for obstacle avoidance
    lidar_emergency_close_mm: float = 300.0 #sets the distance at which the robot will stop if it is too close to an obstacle
    lidar_emergency_debounce_scans: int = 3 #sets the number of scans at which the robot will stop if it is too close to an obstacle (to prevent jittering)
