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
    )
    slam_resolution: float = 0.05
    map_out_dir: str = "maps"
    map_filename_prefix: str = "lidar_map"
    save_map_npy: bool = True
    #exploration_mode "free" ignores waypoints at runtime
