px4_add_module(
	MODULE modules__garrard_fake_mocap
	MAIN garrard_fake_mocap
	SRCS
		garrard_fake_mocap.cpp
		garrard_fake_mocap.hpp
	DEPENDS
		px4_work_queue
)
