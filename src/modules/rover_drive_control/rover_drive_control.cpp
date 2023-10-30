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

		_controller.setInput(_encoder_data, false);

		_output_forwards = _controller.getOutput(false);

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
	_controller.setInput(_input_feed_forward + _input_pid, true);

	_output_inverse = _controller.getOutput(true);

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

	// calculate bearing
	// float desired_heading = computeBearing(_global_position, _current_waypoint);

	// PX4_ERR("new init going into computeadvancedbearing: %f %f", (double)_previous_waypoint(0), (double)_previous_waypoint(1));

	float desired_heading = computeAdvancedBearing(_global_position, _current_waypoint, _previous_waypoint);

	float distance_to_next_wp = get_distance_to_next_waypoint(_global_position(0), _global_position(1),
								  _current_waypoint(0), _current_waypoint(1));

	float heading_error = normalizeAngle(desired_heading - vehicle_yaw);
	// PX4_ERR("heading error %f", (double)heading_error);
	// float heading_error = 0;

	float align_error = computeAlignment(_global_position, _current_waypoint, _previous_waypoint);
	// float align_error = 0;

	_dt = getDt();

	float desired_angular_rate = _yaw_rate_point_pid.pid(heading_error, 0, _dt, 200, true, 2, 0.4, 0) + _yaw_rate_align_pid.pid(align_error, 0, _dt, 200, true, 1, 0.2, 0);

	// PX4_ERR("Desired angular rate %f", (double)desired_angular_rate);

	// float desired_linear_speed = computeDesiredSpeed(distance_to_next_wp);
	float desired_linear_velocity = 2;

	//// Temporary //////////////////////////////////////////////////////////////////////////////////////////

	_forwards_velocity_smoothing.setMaxJerk(22);
	_forwards_velocity_smoothing.setMaxAccel(2);
	_forwards_velocity_smoothing.setMaxVel(4);

	// const float velocity_error_sign = matrix::sign(velocity_error);
	const float max_velocity = math::trajectory::computeMaxSpeedFromDistance(22, 1, distance_to_next_wp, 0.0f);

	_forwards_velocity_smoothing.updateDurations(max_velocity);

	_forwards_velocity_smoothing.updateTraj(_dt);

	desired_linear_velocity = _forwards_velocity_smoothing.getCurrentVelocity();

	// PX4_ERR("Test linear vel sp and real: %f %f", (double)desired_linear_velocity, (double)_forwards_velocity);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////

	_input_feed_forward(0) = desired_linear_velocity;

	_input_feed_forward(1) = desired_angular_rate;
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

// float RoverDriveControl::computeAlignment(const matrix::Vector2f& current_pos, const matrix::Vector2f& waypoint, const matrix::Vector2f& previous_waypoint)
// {
//     matrix::Vector2f wanted_path = waypoint - previous_waypoint;
//     matrix::Vector2f current_path = current_pos - previous_waypoint;

//     // Normalize the vectors
//     wanted_path.normalize();
//     current_path.normalize();

//     float result = -(1 - wanted_path.dot(current_path));

//     // Check if result is finite, and return 0 if not
//     if (!PX4_ISFINITE(result)) {
//         return 0.0f;
//     }

//     return result;
// }


float RoverDriveControl::computeDesiredSpeed(float distance) {
        // You may want to tune these parameters
        float max_speed = 1.0;  // Max linear speed
        float speed_scaling_factor = 1.0;  // How quickly the speed decreases with distance
        return max_speed * std::exp(-speed_scaling_factor * distance);
}

// float RoverDriveControl::computeBearing(const matrix::Vector2f& current_pos, const matrix::Vector2f& waypoint) {
//         float delta_x = waypoint(0) - current_pos(0);
//         float delta_y = waypoint(1) - current_pos(1);
//         return std::atan2(delta_y, delta_x);
// }

// float RoverDriveControl::computeAdvancedBearing(const matrix::Vector2f& current_pos, const matrix::Vector2f& waypoint, const matrix::Vector2f& previous_waypoint) {

// 	// PX4_ERR("two waypoints: %f %f", (double)previous_waypoint(0), (double)previous_waypoint(1));

//         matrix::Vector2f wanted_path = waypoint - previous_waypoint;
// 	matrix::Vector2f current_path = current_pos - previous_waypoint;

// 	// Normalize the vectors
// 	matrix::Vector2f wanted_path_normalized = wanted_path;
// 	matrix::Vector2f current_path_normalized = current_path;

// 	wanted_path_normalized.normalize();
// 	current_path_normalized.normalize();

// 	float dot = wanted_path_normalized.dot(current_path_normalized);

// 	float theta = acos(dot);

// 	matrix::Vector2f p1 = wanted_path_normalized * cos(theta) * current_path.norm() + previous_waypoint;

// 	matrix::Vector2f v1 = current_pos - p1;

// 	matrix::Vector2f new_waypoint = -v1*10 + waypoint;

// 	// PX4_ERR("two waypoints: %f %f and %f %f", (double)current_pos(0), (double)current_pos(1), (double)new_waypoint(0), (double)new_waypoint(1));

// 	return computeBearing(current_pos, new_waypoint);
// }

// float RoverDriveControl::normalizeAngle(float angle) {
// 	// wtf, i hope no one sees this for now lol
//         while (angle > (float)M_PI) angle -= (float)2.0 * (float)M_PI;
//         while (angle < -(float)M_PI) angle += (float)2.0 * (float)M_PI;
//         return angle;
// }

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
