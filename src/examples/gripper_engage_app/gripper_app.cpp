/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "gripper_app.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

#include <uORB/topics/actuator_controls.h>

int GripperApp::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int GripperApp::custom_command(int argc, char *argv[])
{
	
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	/* set engage/disengage flag to true 
	*/
	if (!strcmp(argv[0], "engage")) {
		get_instance()->engage_gripper();
		return 0;
	} else if (!strcmp(argv[0], "disengage")) {
		get_instance()->disengage_gripper();
		return 0;
	}

	return print_usage("Command to engage or disengage gripper");
}


int GripperApp::engage_gripper()
{	
	flag_engage = true;
	flag_disengage = false;
	return 0;
}

int GripperApp::disengage_gripper()
{	
	flag_disengage = true;
	flag_engage = false;
	flag_elapsed_time = false;
	return 0;
}

int GripperApp::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT-5,
				      1200,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

GripperApp *GripperApp::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	GripperApp *instance = new GripperApp(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

GripperApp::GripperApp(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void GripperApp::run()
{

	// initialize parameters
	parameters_update(true);

	struct actuator_controls_s aux;

	while (!should_exit()) {

		usleep(20000);

		/* Set servo positions

		Set the gripper to max,min or neutral position in (-1,1) if no engage/disengage event happen.

		NOTE: -0.95 == 1025, 1 == 2000, -1 = 1000, 0.85 == 1925

		*/

		if (flag_engage) {

			/* send max PWM to activate */

			aux.timestamp = hrt_absolute_time();
			aux.control[1] = max;

			flag_is_engaged = true;

		} else if (flag_disengage) {

			if (flag_is_engaged) {

				/* set min PWM to recoil, then go to neutral after "event_duration" seconds */

				if (!flag_elapsed_time) {
					clock_gettime(CLOCK_REALTIME, &start);
					flag_elapsed_time = true;
				} else {
					clock_gettime(CLOCK_REALTIME, &end);
					elapsed_duration = (double)(end.tv_sec - start.tv_sec);
				}

				if (elapsed_duration > event_duration) {
					aux.timestamp = hrt_absolute_time();
					aux.control[1] = neutral;
				} else {
					aux.timestamp = hrt_absolute_time();
					aux.control[1] = min;
				}
			} else {
				PX4_WARN("Gripper has to be ENGAGED in order to disengage, unable to DISENGAGE");
				flag_disengage = false;
			}

		} else {
			aux.timestamp = hrt_absolute_time();
			aux.control[1] = neutral;
		}

		if (_aux_pub != nullptr) {
			orb_publish(ORB_ID(actuator_controls_2), _aux_pub, &aux);

		} else {
			_aux_pub = orb_advertise(ORB_ID(actuator_controls_2), &aux);
		}

		}

		parameters_update();
	}

// 	orb_unsubscribe(sensor_combined_sub);

// }

void GripperApp::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int GripperApp::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int gripper_app_main(int argc, char *argv[])
{
	return GripperApp::main(argc, argv);
}
