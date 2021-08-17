#include "garrard_mocap_pose_estimator.hpp"

GarrardMocapPoseEstimator::GarrardMocapPoseEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::SPI0)
{
}

GarrardMocapPoseEstimator::~GarrardMocapPoseEstimator()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool GarrardMocapPoseEstimator::init()
{
	// execute Run() on every sensor_accel publication
	if (!_mocap_sub.registerCallback()) {
		PX4_ERR("_mocap_sub callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(10000_us);

	return true;
}

void GarrardMocapPoseEstimator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	#ifdef DEBUG
		struct orb_test_s data;
	#endif

	if (_mocap_sub.updated())
	{
		if(_attitude_sub.copy(&att) && _mocap_sub.copy(&mocap_odom))
		{
			//****************Position and heading data**************//
			// micrortps should send data in correct frame. no negatives needed
			local_pose.x = mocap_odom.x;
			local_pose.y = mocap_odom.y;
			local_pose.z = mocap_odom.z;

			// Yaw angle. https://stackoverflow.com/a/5783030
			// auto q = mocap_odom.q;
			// q[1] = 0;
			// q[3] = 0;
			// float mag = sqrt(q[0]*q[0] + q[2]*q[2]);
			// q[0] /= mag;
			// q[2] /= mag;
			// float ang = 2*acos(q[0]);

			// // local_pose.heading = matrix::Eulerf(matrix::Quatf(mocap_odom.q)).psi();
			// local_pose.heading = ang;

			//************Velocity estimation**************//
			// Shift position arrays
			x_k[2] = x_k[1];
			x_k[1] = x_k[0];
			y_k[2] = y_k[1];
			y_k[1] = y_k[0];
			z_k[2] = z_k[1];
			z_k[1] = z_k[0];

			// Append new position data
			x_k[0] = mocap_odom.x;
			y_k[0] = mocap_odom.y;
			z_k[0] = mocap_odom.z;

			// Shift velocity arrays
			vx_k[2] = vx_k[1];
			vx_k[1] = vx_k[0];
			vy_k[2] = vy_k[1];
			vy_k[1] = vy_k[0];
			vz_k[2] = vz_k[1];
			vz_k[1] = vz_k[0];

			// Compute time period
			time_current = hrt_absolute_time();
			float T = (float)((time_current - time_prev)/1000000.0);
			time_prev = time_current;
			local_pose.timestamp = time_current;
			local_pose.timestamp_sample = (uint64_t)((double)(T)*1000000.0);

			// Get values from parameters
			float wx = _param_wx.get();
			float wy = _param_wy.get();
			float wz = _param_wz.get();
			float xx = _param_xx.get();
			float xy = _param_xy.get();
			float xz = _param_xz.get();

			// Estimate velocity
			vx_k[0] = 	((2*T*wx*wx*x_k[0])
					-(2*T*wx*wx*x_k[2])
					-((4-(4*T*xx*wx)+(T*T*wx*wx))*vx_k[2])
					-(((2*T*T*wx*wx)-8)*vx_k[1]))
					/
					(4+(4*T*xx*wx)+(T*T*wx*wx));
			vy_k[0] = 	(2*T*wy*wy*y_k[0]
					-2*T*wy*wy*y_k[2]
					-(4-4*T*xy*wy+T*T*wy*wy)*vy_k[2]
					-(2*T*T*wy*wy-8)*vy_k[1])
					/
					(4+4*T*xy*wy+T*T*wy*wy);
			vz_k[0] = 	(2*T*wz*wz*z_k[0]
					-2*T*wz*wz*z_k[2]
					-(4-4*T*xz*wz+T*T*wz*wz)*vz_k[2]
					-(2*T*T*wz*wz-8)*vz_k[1])
					/
					(4+4*T*xz*wz+T*T*wz*wz);

			local_pose.vx = vx_k[0];
			local_pose.vy = vy_k[0];
			local_pose.vz = vz_k[0];

			//********MISC
			local_pose.xy_valid = true;
			local_pose.z_valid = true;
			local_pose.v_xy_valid = true;
			local_pose.v_z_valid = true;

			_local_pose_pub.publish(local_pose);

			#ifdef DEBUG
				data.val = T;
				data.timestamp = hrt_absolute_time();
				_orb_test_pub.publish(data);
			#endif
		}
		else
		{
			#ifdef DEBUG
				data.val = 2;
				data.timestamp = hrt_absolute_time();
				_orb_test_pub.publish(data);
			#endif
		}

	}
	else
	{
		#ifdef DEBUG
			data.val = 3;
			data.timestamp = hrt_absolute_time();
			_orb_test_pub.publish(data);
		#endif
	}


	// Example
	//  update vehicle_status to check arming state
	// if (_vehicle_status_sub.updated()) {
	// 	vehicle_status_s vehicle_status;

	// 	if (_vehicle_status_sub.copy(&vehicle_status)) {

	// 		const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	// 		if (armed && !_armed) {
	// 			PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

	// 		} else if (!armed && _armed) {
	// 			PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
	// 		}

	// 		_armed = armed;
	// 	}
	// }


	// Example
	//  grab latest accelerometer data
	// if (_sensor_accel_sub.updated()) {
	// 	sensor_accel_s accel;

	// 	if (_sensor_accel_sub.copy(&accel)) {
	// 		// DO WORK

	// 		// access parameter value (SYS_AUTOSTART)
	// 		if (_param_sys_autostart.get() == 1234) {
	// 			// do something if SYS_AUTOSTART is 1234
	// 		}
	// 	}
	// }


	// Example
	//  publish some data
	// orb_test_s data{};
	// data.val = 314159;
	// data.timestamp = hrt_absolute_time();
	// _orb_test_pub.publish(data);


	perf_end(_loop_perf);
}

int GarrardMocapPoseEstimator::task_spawn(int argc, char *argv[])
{
	GarrardMocapPoseEstimator *instance = new GarrardMocapPoseEstimator();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int GarrardMocapPoseEstimator::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int GarrardMocapPoseEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GarrardMocapPoseEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("garrard_mocap_pose_estimator", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int garrard_mocap_pose_estimator_main(int argc, char *argv[])
{
	return GarrardMocapPoseEstimator::main(argc, argv);
}
