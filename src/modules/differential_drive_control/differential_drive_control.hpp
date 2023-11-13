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

#pragma once

// PX4 includes
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// uORB includes
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>

// Standard library includes
#include <math.h>

// Local includes
#include "differential_drive_control_kinematics.hpp"

namespace differential_drive_control
{

class DifferentialDriveControl : public ModuleBase<DifferentialDriveControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DifferentialDriveControl();
	~DifferentialDriveControl() override = default;

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
	void vehicle_control_mode_poll();
	float get_dt();

	uORB::PublicationMulti<actuator_motors_s> _outputs_pub{ORB_ID(actuator_motors)};

	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	vehicle_control_mode_s _vehicle_control_mode{};
	actuator_motors_s _actuator_motors{};

	differential_drive_control_kinematics _differential_kinematics_controller;

	matrix::Vector2f _input_feed_forward{0.0f, 0.0f};  // _input_feed_forward[0] -> Vx [m/s], _input_feed_forward[1] -> Omega [rad/s]
	matrix::Vector2f _output_inverse{0.0f, 0.0f}; // _output[0] -> Right Motor [rad/s], _output[1] -> Left Motor [rad/s]
	matrix::Vector2f _output_forwards{0.0f, 0.0f}; // _output[0] -> Right Motor [rad/s], _output[1] -> Left Motor [rad/s]

	float _forwards_velocity{0.0f};
	float _angular_velocity{0.0f};

	float _dt{0.0};
	float _last_timestamp{0.0};
	float _current_timestamp{0.0};

	uint8_t _arming_state{0};
	bool _system_calibrating{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RDC_MAX_FORW_VEL>) _param_rdc_max_forwards_velocity,
		(ParamFloat<px4::params::RDC_MAX_ANG_VEL>) _param_rdc_max_angular_velocity,
		(ParamFloat<px4::params::RDC_WHEEL_BASE>) _param_rdc_wheel_base,
		(ParamFloat<px4::params::RDC_WHEEL_RADIUS>) _param_rdc_wheel_radius,
		(ParamInt<px4::params::RDC_WHEEL_COUNT>) _param_rdc_wheel_count,
		(ParamInt<px4::params::RDC_ENCODER>) _param_rdc_encoder_count

	)
};

} // namespace differential_drive_control
