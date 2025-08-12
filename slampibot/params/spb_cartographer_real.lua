
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  odom_frame = "odom", # Use odom frame from OpenCR
  tracking_frame = "imu_link", # Use IMU link as tracking frame
  published_frame = "odom",
  base_frame = "dummy_link", # Your robot's base link
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_submaps = 5,
  num_odometry_states = 1000,
  num_trajectory_data = 1000,
  num_imu_data = 1000,
  num_fixed_frame_pose_data = 1000,
  num_landmark_data = 1000,
  
  # Use IMU data from OpenCR
  use_imu_data = true,
  imu_gravity_time_constant = 10.0,
  pose_graph = {
    constraint_builder = {
      sampling_ratio = 0.03,
      max_constraint_distance = 8.0,
      min_score = 0.55,
      global_localization_min_score = 0.6,
      loop_closure_translation_weight = 1e5,
      loop_closure_rotation_weight = 1e5,
      log_matches = true,
    },
    optimization_problem = {
      huber_scale = 1e-1,
      acceleration_weight = 1e3,
      rotation_weight = 1e4,
      fixed_frame_pose_translation_weight = 1e4,
      fixed_frame_pose_rotation_weight = 1e4,
      local_slam_pose_translation_weight = 1e5,
      local_slam_pose_rotation_weight = 1e5,
      odometry_translation_weight = 1e5,
      odometry_rotation_weight = 1e5,
      imu_acceleration_weight = 1e5,
      imu_rotation_weight = 1e5,
      max_num_iterations = 50,
      use_ceres_solver = true,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    matcher_translation_weight = 1e5,
    matcher_rotation_weight = 1e5,
  },
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

MAP_BUILDER.use_trajectory_builder_3d = false

MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = options.use_imu_data
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.acceleration_weight = 1e2
POSE_GRAPH.optimization_problem.rotation_weight = 1e3

return options
