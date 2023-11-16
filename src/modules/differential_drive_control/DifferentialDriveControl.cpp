/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "DifferentialDriveControl.hpp"

using namespace time_literals;

namespace differential_drive_control
{

DifferentialDriveControl::DifferentialDriveControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_outputs_pub.advertise();
	_last_timestamp = hrt_absolute_time();
	_differential_drive_kinematics.setWheelBase(_param_rdd_wheel_base.get());
	_differential_drive_kinematics.setWheelRadius(_param_rdd_wheel_radius.get());
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

	obj->start();

	return 0;
}

void DifferentialDriveControl::start()
{
	ScheduleOnInterval(10_ms); // 100 Hz
}

void DifferentialDriveControl::Run()
{

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	_current_timestamp = hrt_absolute_time();

	vehicle_control_mode_poll();

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			if (_arming_state != vehicle_status.arming_state) {
				_arming_state = vehicle_status.arming_state;
			}
			bool system_calibrating = vehicle_status.calibration_enabled;

			if (system_calibrating != _system_calibrating) {
				_system_calibrating = system_calibrating;
			}
		}
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();
	}


	if(_vehicle_control_mode.flag_control_manual_enabled && _vehicle_control_mode.flag_armed){
		if (_manual_control_setpoint_sub.updated()) {
			manual_control_setpoint_s manual_control_setpoint{};
			_manual_control_setpoint_sub.copy(&manual_control_setpoint);

			// directly get the input from the manual control setpoint (joystick)
			_input_feed_forward(0) = manual_control_setpoint.throttle*_param_rdd_max_forwards_velocity.get();
			_input_feed_forward(1) = manual_control_setpoint.roll*_param_rdd_max_angular_velocity.get();
		}
	} else {
		// if the system is in an error state, stop the vehicle
		_input_feed_forward = {0.0f, 0.0f};
	}

	// get the wheel speeds from the inverse kinematics class (DifferentialDriveKinematics)
	_differential_drive_kinematics.setInput(_input_feed_forward, true);
	_output_inverse = _differential_drive_kinematics.getOutput(true);

	// publish data to actuator_motors (output module)
	publishRateControl();

	_last_timestamp = _current_timestamp;

}

void DifferentialDriveControl::publishRateControl()
{
	// Superpose Linear and Angular velocity vector
	float max_angular_wheel_speed = ((_param_rdd_max_forwards_velocity.get() + (_param_rdd_max_angular_velocity.get()*_param_rdd_wheel_base.get()/2)) / _param_rdd_wheel_radius.get());

	_actuator_motors.timestamp = hrt_absolute_time();
	_actuator_motors.reversible_flags = 3;
	_actuator_motors.control[0] = _output_inverse(0)/max_angular_wheel_speed;
	_actuator_motors.control[1] = _output_inverse(1)/max_angular_wheel_speed;

	_outputs_pub.publish(_actuator_motors);
}

void DifferentialDriveControl::vehicle_control_mode_poll()
{
	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}
}

int DifferentialDriveControl::print_status()
{
	PX4_INFO("Diffential Drive Controller running");
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
Rover Differential Drive controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("differential_drive", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int differential_drive_control_main(int argc, char *argv[])
{
	return DifferentialDriveControl::main(argc, argv);
}

} // namespace differential_drive_control
