-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 5e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.,

}

--MAX_3D_RANGE = 100
--TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
--TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.02
--TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 15
--TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.20

-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5e5
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 8e5
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5e-1
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 8e-1
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 300

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 8
POSE_GRAPH.optimize_every_n_nodes = 200
POSE_GRAPH.constraint_builder.sampling_ratio = 0.02
--POSE_GRAPH.global_sampling_ratio = 0.05

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10

--POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
--POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2
--POSE_GRAPH.optimization_problem.acceleration_weight = 1e2
--POSE_GRAPH.optimization_problem.rotation_weight = 1e3
--POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
--POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2
--POSE_GRAPH.global_constraint_search_after_n_seconds = 5.0

POSE_GRAPH.constraint_builder.min_score = 0.4
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.3


return options
