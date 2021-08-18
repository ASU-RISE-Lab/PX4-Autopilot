#pragma once

//#define DEBUG

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>

#ifdef DEBUG
	#include <uORB/topics/orb_test.h>
#endif


using namespace time_literals;

class GarrardMocapPoseEstimator : public ModuleBase<GarrardMocapPoseEstimator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	GarrardMocapPoseEstimator();
	~GarrardMocapPoseEstimator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	float x_k[3] = {};
	float y_k[3] = {};
	float z_k[3] = {};
	float vx_k[3] = {};
	float vy_k[3] = {};
	float vz_k[3] = {};
	float wx, wy, wz, xx, xy, xz;
	uint64_t time_prev = hrt_absolute_time()/1000000.0;;
	uint64_t time_current;
	struct vehicle_odometry_s mocap_odom = {};
	struct vehicle_local_position_s local_pose = {};
	struct vehicle_attitude_s att = {};

	// Publications
	#ifdef DEBUG
		uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	#endif
	uORB::Publication<vehicle_local_position_s> _local_pose_pub{ORB_ID(vehicle_local_position)};

	// Subscriptions
	// uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules this work item when updated
	// uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::SubscriptionCallbackWorkItem _mocap_sub{this, ORB_ID(vehicle_mocap_odometry)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_OMEGA_X>) _param_wx,
		(ParamFloat<px4::params::MC_OMEGA_Y>) _param_wy,
		(ParamFloat<px4::params::MC_OMEGA_Z>) _param_wz,
		(ParamFloat<px4::params::MC_XI_X>) _param_xx,
		(ParamFloat<px4::params::MC_XI_Y>) _param_xy,
		(ParamFloat<px4::params::MC_XI_Z>) _param_xz
	)


	bool _armed{false};
};
