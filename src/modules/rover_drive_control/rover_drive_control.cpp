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

#include "rover_drive_control.hpp"

using namespace time_literals;
using matrix::Vector3f;

namespace rover_drive_control
{

RoverDriveControl::RoverDriveControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_differential_drive_control_pub.advertise();
	_last_timestamp = hrt_absolute_time();
}

RoverDriveControl::~RoverDriveControl()
{
}

int RoverDriveControl::task_spawn(int argc, char *argv[])
{
	RoverDriveControl *obj = new RoverDriveControl();

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

void RoverDriveControl::encoder_data_poll()
{
	// TODO Add parameters that asks how many wheels there are.

	float sum_left_encoders_speed = 0.0f;
	float sum_right_encoders_speed = 0.0f;
	int count_left_encoders = 0;
	int count_right_encoders = 0;

	for (int i = 0; i < _wheel_encoders_sub.size(); ++i) {
		auto &sub = _wheel_encoders_sub[i];
		if (sub.update(&_wheel_encoder)) {
		if (i < 2) {  // Assuming first two encoders are for the right side
			sum_right_encoders_speed += _wheel_encoder.speed;
			count_right_encoders++;
		} else {  // Assuming the last two encoders are for the left side
			sum_left_encoders_speed += _wheel_encoder.speed;
			count_left_encoders++;
		}
		}
	}

	if (count_right_encoders > 0 && count_left_encoders > 0) {
		_encoder_data(0) = sum_right_encoders_speed / count_right_encoders;
		_encoder_data(1) = sum_left_encoders_speed / count_left_encoders;

		_kinematics_controller.setInput(_encoder_data, false);

		_output_forwards = _kinematics_controller.getOutput(false);

	}
	else {
		PX4_ERR("No encoder data received.");
	}
}


void RoverDriveControl::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		_control_mode_sub.copy(&_control_mode);
	}
}

void RoverDriveControl::position_setpoint_triplet_poll()
{
	if (_pos_sp_triplet_sub.updated()) {
		_pos_sp_triplet_sub.copy(&_pos_sp_triplet);
	}
}

void RoverDriveControl::vehicle_position_poll()
{
	if(_global_pos_sub.updated()) {
		_global_pos_sub.copy(&_global_pos);
	}


	if(_local_pos_sub.updated()) {
		_local_pos_sub.copy(&_local_pos);
	}

}

void RoverDriveControl::vehicle_attitude_poll()
{
	if (_att_sub.updated()) {
		_att_sub.copy(&_vehicle_att);
	}
}

void RoverDriveControl::start()
{
	ScheduleOnInterval(10_ms); // 100 Hz
}

void RoverDriveControl::Run()
{

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	_current_timestamp = hrt_absolute_time();

	vehicle_control_mode_poll();
	position_setpoint_triplet_poll();
	vehicle_position_poll();
	vehicle_attitude_poll();
	getLocalVelocity();
	encoder_data_poll();
	_dt = getDt();

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

	if(_control_mode.flag_control_manual_enabled && _control_mode.flag_armed){
		subscribeManualControl();
	} else if (_control_mode.flag_control_auto_enabled && _control_mode.flag_armed) {

		subscribeAutoControl();
	} else {
		_input_pid = {0.0f, 0.0f};
		_input_feed_forward = {0.0f, 0.0f};
	}

	/////// Fix this section, does not work well with the inverse bool
	_kinematics_controller.setInput(_input_feed_forward + _input_pid, true);

	_output_inverse = _kinematics_controller.getOutput(true);

	publishRateControl();

	//////////////////////////////////////////////////////////////////

	_last_timestamp = _current_timestamp;

}

void RoverDriveControl::subscribeManualControl()
{
	_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

	_input_feed_forward(0) = _manual_control_setpoint.throttle*100;
	_input_feed_forward(1) = _manual_control_setpoint.roll*20;
}

void RoverDriveControl::getLocalVelocity()
{
	Vector2f velocity_vector{0.0, 0.0};

	velocity_vector(0) = _local_pos.vx;
	velocity_vector(1) = _local_pos.vy;

	_forwards_velocity = velocity_vector.norm();
}

void RoverDriveControl::subscribeAutoControl()
{

	_global_position(0) = _global_pos.lat;
	_global_position(1) = _global_pos.lon;

	_next_waypoint(0) = _pos_sp_triplet.next.lat;
	_next_waypoint(1) = _pos_sp_triplet.next.lon;

	_current_waypoint(0) = _pos_sp_triplet.current.lat;
	_current_waypoint(1) = _pos_sp_triplet.current.lon;

	if(!PX4_ISFINITE(_pos_sp_triplet.previous.lat) && !_intialized){
		// PX4_ERR("fixing bad initialization");
		_previous_waypoint(0) = _global_position(0);
		_previous_waypoint(1) = _global_position(1);
		// PX4_ERR("new init: %f %f", (double)_previous_waypoint(0), (double)_previous_waypoint(1));
		_intialized = true;
	} else if (PX4_ISFINITE(_pos_sp_triplet.previous.lat)){
		_previous_waypoint(0) = _pos_sp_triplet.previous.lat;
		_previous_waypoint(1) = _pos_sp_triplet.previous.lon;
	}

	const float vehicle_yaw = matrix::Eulerf(matrix::Quatf(_vehicle_att.q)).psi();

	matrix::Vector2f guidance_output = _guidance_controller.computeGuidance(_global_position, _current_waypoint, _previous_waypoint, _next_waypoint, vehicle_yaw, _dt);

	_input_feed_forward(0) = guidance_output(0);

	_input_feed_forward(1) = guidance_output(1);

}

//temporary
void RoverDriveControl::publishAllocation()
{
	vehicle_thrust_setpoint_s v_thrust_sp{};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.xyz[0] = _manual_control_setpoint.throttle;
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = 0.0f;
	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);

	vehicle_torque_setpoint_s v_torque_sp{};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.xyz[0] = 0.f;
	v_torque_sp.xyz[1] = 0.f;
	v_torque_sp.xyz[2] = _manual_control_setpoint.roll;
	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

float RoverDriveControl::getDt()
{

	return ((_current_timestamp - _last_timestamp)/1000000);
}

void RoverDriveControl::publishRateControl()
{
	// magnetometer_bias_estimate_s mag_bias_est{};
	differential_drive_control_s diff_drive_control{};

	diff_drive_control.motor_control[0] = _output_inverse(0);
	diff_drive_control.motor_control[1] = _output_inverse(1);

	diff_drive_control.linear_velocity[0] = _output_forwards(0);
	diff_drive_control.angular_velocity[2] = _output_forwards(1);

	diff_drive_control.timestamp = hrt_absolute_time();

	_differential_drive_control_pub.publish(diff_drive_control);

}

int RoverDriveControl::print_status()
{

	return 0;
}

int RoverDriveControl::print_usage(const char *reason)
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

extern "C" __EXPORT int rover_drive_control_main(int argc, char *argv[])
{
	return RoverDriveControl::main(argc, argv);
}

} // namespace load_mon
