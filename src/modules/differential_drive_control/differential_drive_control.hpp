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


#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_drive_control.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>


namespace differential_drive_control
{

class DifferentialDriveControl : public ModuleBase<DifferentialDriveControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DifferentialDriveControl();
	~DifferentialDriveControl() override;

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

	differential_drive_control_s 		_diff_drive_control{};
	manual_control_setpoint_s		_manual_control_setpoint{};
	vehicle_control_mode_s			_control_mode{};
	actuator_outputs_s 			_actuator_outputs{};

	differential_drive_control_kinematics _controller;

	matrix::Vector2f _input_pid{0.0f, 0.0f};  // input_[0] -> Vx [m/s], input_[1] -> Omega [rad/s]
	matrix::Vector2f _input_feed_forward{0.0f, 0.0f};  // _input_feed_forward[0] -> Vx [m/s], _input_feed_forward[1] -> Omega [rad/s]
	matrix::Vector2f _output{0.0f, 0.0f}; // _output[0] -> Right Motor [rad/s], _output[1] -> Left Motor [rad/s]

	uint8_t _arming_state{0};
	bool _system_calibrating{false};

	// temporary

	// gz::transport::Node::Publisher _actuators_pub;

};

} // namespace differential_drive_control
