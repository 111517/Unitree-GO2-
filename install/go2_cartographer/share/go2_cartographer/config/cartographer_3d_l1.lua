include "map_builder.lua"
include "trajectory_builder_3d.lua"

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER = MAP_BUILDER.trajectory_builder_3d

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "base_link",
  odom_frame = "odom",

  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_point_clouds = 1,

  lookup_transform_timeout_sec = 0.3,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  use_pose_extrapolator = true,
}

-- Trajectory Builder 3D
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 25.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.10
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter = {
  max_length = 0.2,
  min_num_points = 200,
  max_range = 20.0,
}

TRAJECTORY_BUILDER_3D.motion_filter = {
  max_time_seconds = 0.5,
  max_distance_meters = 0.1,
  max_angle_radians = 0.1,
}

TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.0
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45

-- Pose Graph 配置可以直接写，不需要再次 include pose_graph.lua
POSE_GRAPH.optimize_every_n_nodes = 80
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2
POSE_GRAPH.constraint_builder.min_score = 0.50
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

return options
