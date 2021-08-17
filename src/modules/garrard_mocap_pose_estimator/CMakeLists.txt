px4_add_module(
	MODULE modules__garrard_mocap_pose_estimator
	MAIN garrard_mocap_pose_estimator
	SRCS
		garrard_mocap_pose_estimator.cpp
		garrard_mocap_pose_estimator.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
)
