include "carto_hw_slam.lua"
options.rangefinder_sampling_ratio = 0.5
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 1000,
}
--TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 100
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 300
--POSE_GRAPH.optimization_problem.acceleration_weight = 0.01
--POSE_GRAPH.optimization_problem.rotation_weight = 1
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.constraint_builder.sampling_ratio = 0.01
POSE_GRAPH.global_sampling_ratio = 0.005
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45
--POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 15
return options
