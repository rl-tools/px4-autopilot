/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "differential_drive_control.hpp"

using namespace time_literals;
using matrix::Vector3f;

namespace differential_drive_control
{

DifferentialDriveControl::DifferentialDriveControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_differential_drive_control_pub.advertise();
}

DifferentialDriveControl::~DifferentialDriveControl()
{
}

int DifferentialDriveControl::task_spawn(int argc, char *argv[])
{
	DifferentialDriveControl *obj = new DifferentialDriveControl();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	/* Schedule a cycle to start things. */
	obj->start();

	return 0;
}

void DifferentialDriveControl::start()
{
	ScheduleOnInterval(20_ms); // 50 Hz
}

void DifferentialDriveControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			if (_arming_state != vehicle_status.arming_state) {
				_arming_state = vehicle_status.arming_state;

			}

			// check this
			bool system_calibrating = vehicle_status.calibration_enabled;

			if (system_calibrating != _system_calibrating) {
				_system_calibrating = system_calibrating;

			}
		}
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	if(true){ // change this to manual flag
		subscribeManualControl();
	}

	_controller.setInput(_input);

	_controller.computeControl();

	_output = _controller.getOutput();

	publishRateControl();


}

void DifferentialDriveControl::subscribeManualControl()
{
	_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

	_input(0) = _manual_control_setpoint.throttle;
	_input(1) = _manual_control_setpoint.roll;
}

void DifferentialDriveControl::publishRateControl()
{
	// magnetometer_bias_estimate_s mag_bias_est{};
	differential_drive_control_s diff_drive_control{};

	diff_drive_control.motor_control[0] = _output(0);
	diff_drive_control.motor_control[1] = _output(1);

	diff_drive_control.timestamp = hrt_absolute_time();

	_differential_drive_control_pub.publish(diff_drive_control);

}

int DifferentialDriveControl::print_status()
{
	// for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
	// 	if (_calibration[mag_index].device_id() != 0) {

	// 		_calibration[mag_index].PrintStatus();

	// 		const Vector3f &bias = _bias_estimator[mag_index].getBias();

	// 		PX4_INFO("%d (%" PRIu32 ") bias: [% 05.3f % 05.3f % 05.3f]",
	// 			 mag_index, _calibration[mag_index].device_id(),
	// 			 (double)bias(0),
	// 			 (double)bias(1),
	// 			 (double)bias(2));
	// 	}
	// }

	return 0;
}

int DifferentialDriveControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Online magnetometer bias estimator.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mag_bias_estimator", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int differential_drive_control_main(int argc, char *argv[])
{
	return DifferentialDriveControl::main(argc, argv);
}

} // namespace load_mon
