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

#pragma once

#include "differential_drive_control_kinematics.hpp"
#include "differential_drive_control_guidance.hpp"
#include "rover_drive_control_pid.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/actuator_outputs.h>
#include <uORB/PublicationMulti.hpp>

// //temporary
// #include <gz/transport/Node.hh>
// #include <gz/msgs/twist.pb.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>

#include <lib/geo/geo.h>
#include <math.h>

#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>

#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_drive_control.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/wheel_encoders.h>


namespace rover_drive_control
{

class RoverDriveControl : public ModuleBase<RoverDriveControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	RoverDriveControl();
	~RoverDriveControl() override;

	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void start();

private:

	void Run() override;
	void publishRateControl();
	void subscribeManualControl();
	void subscribeAutoControl();
	void vehicle_control_mode_poll();
	void setAndPublishActuatorOutputs();
	void position_setpoint_triplet_poll();
	void vehicle_position_poll();
	void vehicle_attitude_poll();
	void encoder_data_poll();
	float getDt();
	void getLocalVelocity();

	float computeBearing(const matrix::Vector2f& current_pos, const matrix::Vector2f& waypoint);
	float computeAdvancedBearing(const matrix::Vector2f& current_pos, const matrix::Vector2f& waypoint, const matrix::Vector2f& previous_waypoint);
	float normalizeAngle(float angle);
	float computeAlignment(const matrix::Vector2f& current_pos, const matrix::Vector2f& waypoint, const matrix::Vector2f& previous_waypoint);
	float computeDesiredSpeed(float distance);

	// temporary

	void publishAllocation();

	// void publish(double linear_x, double angular_z);

	uORB::Publication<differential_drive_control_s> _differential_drive_control_pub{ORB_ID(differential_drive_control)};
	uORB::PublicationMulti<actuator_outputs_s> _outputs_pub{ORB_ID(actuator_outputs)};

	// temporary

	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};


	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	// uORB::Subscription _encoder_data_sub{ORB_ID(wheel_encoders)};
	uORB::SubscriptionMultiArray<wheel_encoders_s>	_wheel_encoders_sub{ORB_ID::wheel_encoders};

	differential_drive_control_s 		_diff_drive_control{};
	manual_control_setpoint_s		_manual_control_setpoint{};
	vehicle_control_mode_s			_control_mode{};
	actuator_outputs_s 			_actuator_outputs{};
	position_setpoint_triplet_s 		_pos_sp_triplet{};
	vehicle_global_position_s		_global_pos{};			/**< global vehicle position */
	vehicle_local_position_s		_local_pos{};			/**< global vehicle position */
	vehicle_attitude_s			_vehicle_att{};
	wheel_encoders_s 			_wheel_encoder;

	differential_drive_control_kinematics 	_kinematics_controller;
	differential_drive_control_guidance 	_guidance_controller;
	rover_drive_control_pid 		_yaw_rate_point_pid;
	rover_drive_control_pid 		_yaw_rate_align_pid;
	rover_drive_control_pid 		_speed_control_pid;

	matrix::Vector2f _input_pid{0.0f, 0.0f};  // input_[0] -> Vx [m/s], input_[1] -> Omega [rad/s]
	matrix::Vector2f _input_feed_forward{0.0f, 0.0f};  // _input_feed_forward[0] -> Vx [m/s], _input_feed_forward[1] -> Omega [rad/s]
	matrix::Vector2f _output_inverse{0.0f, 0.0f}; // _output[0] -> Right Motor [rad/s], _output[1] -> Left Motor [rad/s]
	matrix::Vector2f _output_forwards{0.0f, 0.0f}; // _output[0] -> Right Motor [rad/s], _output[1] -> Left Motor [rad/s]
	matrix::Vector2f _encoder_data{0.0f, 0.0f};
	float 		 _forwards_velocity{0.0f};

	matrix::Vector2f _global_position{0.0, 0.0};
	matrix::Vector2f _local_position{0.0, 0.0};
	matrix::Vector2f _current_waypoint{0.0, 0.0};
	matrix::Vector2f _previous_waypoint{0.0, 0.0};
	matrix::Vector2f _next_waypoint{0.0, 0.0};
	bool 		 _intialized = false;

	VelocitySmoothing _forwards_velocity_smoothing;
	PositionSmoothing _position_smoothing;

	double _theta{0.0};

	float _dt{0.0};
	float _last_timestamp{0.0};
	float _current_timestamp{0.0};

	uint8_t _arming_state{0};
	bool _system_calibrating{false};

	// temporary

	// gz::transport::Node::Publisher _actuators_pub;

};

} // namespace rover_drive_control
