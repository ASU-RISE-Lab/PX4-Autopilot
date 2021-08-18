#include "garrard_fake_mocap.hpp"

WorkItemExample::WorkItemExample() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::SPI0)
{
}

WorkItemExample::~WorkItemExample()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExample::init()
{
	// alternatively, Run on fixed interval
	ScheduleOnInterval(8333_us); // 120 Hz rate
	return true;
}

void WorkItemExample::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	fake_mocap_odom.x = sin(hrt_absolute_time()/1000000.0);
	fake_mocap_odom.y = cos(hrt_absolute_time()/1000000.0);
	fake_mocap_odom.z = -sin(hrt_absolute_time()/1000000.0);
	fake_mocap_odom.local_frame = 0;
	// fake_mocap_odom.q[0] = sin(hrt_absolute_time()/2500000.0);
	// fake_mocap_odom.q[3] = sqrt(1-(fake_mocap_odom.q[0]*fake_mocap_odom.q[0]));
	fake_mocap_odom.timestamp = hrt_absolute_time();
	fake_mocap_odom.timestamp_sample = fake_mocap_odom.timestamp;

	_mocap_sub.publish(fake_mocap_odom);

	perf_end(_loop_perf);
}

int WorkItemExample::task_spawn(int argc, char *argv[])
{
	WorkItemExample *instance = new WorkItemExample();

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

int WorkItemExample::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExample::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("garrard_fake_mocap", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int garrard_fake_mocap_main(int argc, char *argv[])
{
	return WorkItemExample::main(argc, argv);
}
