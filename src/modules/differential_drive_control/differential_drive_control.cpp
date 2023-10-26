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
	_last_timestamp = hrt_absolute_time();
	// temporary
	// _actuators_pub = _node.Advertise<gz::msgs::Twist>("/model/r1_rover_0/cmd_vel");
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
	// ScheduleOnInterval(1_s);
}

void DifferentialDriveControl::Run()
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
	} else if (_control_mode.flag_control_auto_enabled && _control_mode.flag_armed) { // temporary, unless we wont have more than just these two modes
		subscribeAutoControl();
	} else {
		_input_pid = {0.0f, 0.0f};
		_input_feed_forward = {0.0f, 0.0f};
	}

	_controller.setInput(_input_feed_forward + _input_pid, true);

	_output = _controller.getOutput();

	publishRateControl();

	// temporary
	// publishAllocation();

	// setAndPublishActuatorOutputs();

	_last_timestamp = _current_timestamp;

}

//temporary
void DifferentialDriveControl::publishAllocation()
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

// void
// DifferentialDriveControl::publish(double linear_x, double angular_z) {
//     gz::msgs::Twist msg;

//     msg.mutable_linear()->set_x(linear_x);
//     msg.mutable_angular()->set_z(angular_z);

//     _actuators_pub.Publish(msg);
// }

// void
// DifferentialDriveControl::setAndPublishActuatorOutputs()
// {
//         _actuator_outputs.noutputs = 2;
//         for (size_t i = 0; i < 2; ++i) {
//                 _actuator_outputs.output[i] = _output(i);
//         }
//         _actuator_outputs.timestamp = hrt_absolute_time();
//         _outputs_pub.publish(_actuator_outputs);
// }

void DifferentialDriveControl::vehicle_control_mode_poll()
{
	if (_control_mode_sub.updated()) {
		_control_mode_sub.copy(&_control_mode);
	}
}

void DifferentialDriveControl::position_setpoint_triplet_poll()
{
	if (_pos_sp_triplet_sub.updated()) {
		_pos_sp_triplet_sub.copy(&_pos_sp_triplet);
	}
}

void DifferentialDriveControl::vehicle_position_poll()
{
	if(_global_pos_sub.updated()) {
		_global_pos_sub.copy(&_global_pos);
	}


	if(_local_pos_sub.updated()) {
		_local_pos_sub.copy(&_local_pos);
	}

}

void DifferentialDriveControl::vehicle_attitude_poll()
{
	if (_att_sub.updated()) {
		_att_sub.copy(&_vehicle_att);
	}
}

void DifferentialDriveControl::subscribeManualControl()
{
	_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

	_input_feed_forward(0) = _manual_control_setpoint.throttle*100;
	_input_feed_forward(1) = _manual_control_setpoint.roll*20;

	// PX4_ERR("My control inputs direcly are %f and %f", (double)_input_feed_forward(0), (double)_input_feed_forward(1));
}

void DifferentialDriveControl::getLocalVelocity()
{
	Vector2f velocity_vector{0.0, 0.0};

	velocity_vector(0) = _local_pos.vx;
	velocity_vector(1) = _local_pos.vy;

	_forwards_velocity = velocity_vector.norm();
}

void DifferentialDriveControl::subscribeAutoControl()
{

	_global_position(0) = _global_pos.lat;
	_global_position(1) = _global_pos.lon;

	_next_waypoint(0) = _pos_sp_triplet.next.lat;
	_next_waypoint(1) = _pos_sp_triplet.next.lon;

	_current_waypoint(0) = _pos_sp_triplet.current.lat;
	_current_waypoint(1) = _pos_sp_triplet.current.lon;

	_previous_waypoint(0) = _pos_sp_triplet.previous.lat;
	_previous_waypoint(1) = _pos_sp_triplet.previous.lon;

	const float vehicle_yaw = matrix::Eulerf(matrix::Quatf(_vehicle_att.q)).psi();

	// calculate bearing
	// float desired_heading = computeBearing(_global_position, _current_waypoint);
	float desired_heading = computeAdvancedBearing(_global_position, _current_waypoint, _previous_waypoint);

	float distance_to_next_wp = get_distance_to_next_waypoint(_global_position(0), _global_position(1),
								  _current_waypoint(0), _current_waypoint(1));

	float heading_error = normalizeAngle(desired_heading - vehicle_yaw);
	// float heading_error = 0;

	float align_error = computeAlignment(_global_position, _current_waypoint, _previous_waypoint);
	// float align_error = 0;

	// PX4_ERR("Alighnment error: %f", (double)align_error);
	// PX4_ERR("Heading error: %f", (double)heading_error);


	_dt = getDt();

	float desired_angular_rate = _yaw_rate_point_pid.pid(heading_error, 0, _dt, 200, true, 2, 0.4, 0) + _yaw_rate_align_pid.pid(align_error, 0, _dt, 200, true, 1, 0.2, 0);

	// float desired_linear_speed = computeDesiredSpeed(distance_to_next_wp);
	float desired_linear_velocity = 2;



	// temporary ///////////////////////////////////////////////////////////////////////////////////////////////

	// PositionSmoothing::PositionSmoothingSetpoints smoothed_setpoints;
	// matrix::Vector3f _velocity_setpoint = {2.0, 0.0, 0.0};
	// matrix::Vector3f _previous_waypoint_t = {0.0, 0.0, 0.0};
	// matrix::Vector3f _current_waypoint_t = {0.0, 0.0, 0.0};
	// matrix::Vector3f _next_waypoint_t = {0.0, 0.0, 0.0};
	// matrix::Vector3f _global_position_t = {0.0, 0.0, 0.0};

	// _previous_waypoint_t(0) = _previous_waypoint(0);
	// _previous_waypoint_t(1) = _previous_waypoint(1);

	// _current_waypoint_t(0) = _current_waypoint(0);
	// _current_waypoint_t(1) = _current_waypoint(1);

	// _next_waypoint_t(0) = _next_waypoint(0);
	// _next_waypoint_t(1) = _next_waypoint(1);

	// _global_position_t(0) = _global_position(0);
	// _global_position_t(1) = _global_position(1);

	// Vector3f waypoints[] = {_previous_waypoint_t, _current_waypoint_t, _next_waypoint_t};


	// _position_smoothing.generateSetpoints(
	// 	_global_position_t,
	// 	waypoints,
	// 	_velocity_setpoint,
	// 	_dt,
	// 	false,
	// 	smoothed_setpoints
	// );

	// PX4_ERR("smoothed velocity x: %f, y: %f, z: %f", (double)smoothed_setpoints.velocity(0), (double)smoothed_setpoints.velocity(1), (double)smoothed_setpoints.velocity(2));


	// _velocity_smoothing_x.updateDurations(_velocity_setpoint(0));


	// VelocitySmoothing::timeSynchronization(_velocity_smoothing_x, 3);

	// float test_setpint = _velocity_smoothing_x.getVelSp();

	// PX4_ERR("Test sp: %f", (double)test_setpint);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////

	//// new temporary ////////////////////////////////////////////////////////////////////////////////////////



	_forwards_velocity_smoothing.setMaxJerk(22);
	_forwards_velocity_smoothing.setMaxAccel(2);
	_forwards_velocity_smoothing.setMaxVel(4);

	// const float velocity_error_sign = matrix::sign(velocity_error);
	const float max_velocity = math::trajectory::computeMaxSpeedFromDistance(22, 1, distance_to_next_wp, 0.0f);

	_forwards_velocity_smoothing.updateDurations(max_velocity);

	_forwards_velocity_smoothing.updateTraj(_dt);

	desired_linear_velocity = _forwards_velocity_smoothing.getCurrentVelocity();

	PX4_ERR("Test linear vel sp and real: %f %f", (double)desired_linear_velocity, (double)_forwards_velocity);





	//////////////////////////////////////////////////////////////////////////////////////////////////////////

	_input_feed_forward(0) = desired_linear_velocity;

	_input_feed_forward(1) = desired_angular_rate;
}



float DifferentialDriveControl::getDt()
{

	return ((_current_timestamp - _last_timestamp)/1000000);
}

float DifferentialDriveControl::computeAlignment(const matrix::Vector2f& current_pos, const matrix::Vector2f& waypoint, const matrix::Vector2f& previous_waypoint)
{
    matrix::Vector2f wanted_path = waypoint - previous_waypoint;
    matrix::Vector2f current_path = current_pos - previous_waypoint;

    // Normalize the vectors
    wanted_path.normalize();
    current_path.normalize();

    float result = -(1 - wanted_path.dot(current_path));

    // Check if result is finite, and return 0 if not
    if (!PX4_ISFINITE(result)) {
        return 0.0f;
    }

    return result;
}


float DifferentialDriveControl::computeDesiredSpeed(float distance) {
        // You may want to tune these parameters
        float max_speed = 1.0;  // Max linear speed
        float speed_scaling_factor = 1.0;  // How quickly the speed decreases with distance
        return max_speed * std::exp(-speed_scaling_factor * distance);
}

float DifferentialDriveControl::computeBearing(const matrix::Vector2f& current_pos, const matrix::Vector2f& waypoint) {
        float delta_x = waypoint(0) - current_pos(0);
        float delta_y = waypoint(1) - current_pos(1);
        return std::atan2(delta_y, delta_x);
}

float DifferentialDriveControl::computeAdvancedBearing(const matrix::Vector2f& current_pos, const matrix::Vector2f& waypoint, const matrix::Vector2f& previous_waypoint) {
        matrix::Vector2f wanted_path = waypoint - previous_waypoint;
	matrix::Vector2f current_path = current_pos - previous_waypoint;

	// Normalize the vectors
	matrix::Vector2f wanted_path_normalized = wanted_path;
	matrix::Vector2f current_path_normalized = current_path;

	wanted_path_normalized.normalize();
	current_path_normalized.normalize();

	float dot = wanted_path_normalized.dot(current_path_normalized);

	float theta = acos(dot);

	matrix::Vector2f p1 = wanted_path_normalized * cos(theta) * current_path.norm() + previous_waypoint;

	matrix::Vector2f v1 = current_pos - p1;

	matrix::Vector2f new_waypoint = -v1*10 + waypoint;

	return computeBearing(current_pos, new_waypoint);
}

float DifferentialDriveControl::normalizeAngle(float angle) {
	// wtf, i hope no one sees this for now lol
        while (angle > (float)M_PI) angle -= (float)2.0 * (float)M_PI;
        while (angle < -(float)M_PI) angle += (float)2.0 * (float)M_PI;
        return angle;
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
