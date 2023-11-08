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

/**
 * Wheel Base (Distance from wheel to wheel)
 *
 * This enables continuous calibration of the magnetometers
 * before takeoff using gyro data.
 *
 * @unit m
 * @min 0.0
 * @max 100
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_WHEEL_BASE, 0.54f);

/**
 * Wheel Radius
 *
 * @unit m
 * @min 0.0
 * @max 100
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_WHEEL_RADIUS, 0.0686f);

/**
 * Max Forwards Velocity
 *
 *
 * @unit m/s
 * @min 0.0
 * @max 100
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_MAX_FORW_VEL, 1.0f);

/**
 * Max Angular Velocity
 *
 *
 * @unit rad/s
 * @min 0.0
 * @max 100
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_MAX_ANG_VEL, 1.0f);

/**
 * P Gain Waypoint Controller
 *
 *
 * @min 0.0
 * @max 100
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_P_GAIN_WC, 1.0f);

/**
 * I Gain Waypoint Controller
 *
 *
 * @min 0.0
 * @max 100
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_I_GAIN_WC, 0.0f);

/**
 * D Gain Waypoint Controller
 *
 *
 * @min 0.0
 * @max 100
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_D_GAIN_WC, 0.0f);

/**
 * Waypoint alignment velocity subtraction
 *
 *
 * @min 0.0
 * @max 1
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_VEL_ALGN, 0.2f);

/**
 * Max linear velocity Jerk
 *
 *
 * @unit m/s^3
 * @min 0.0
 * @max 100
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_MAX_JERK, 22.f);

/**
 * Max linear acceleration
 *
 *
 * @unit m/s^2
 * @min 0.0
 * @max 100
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_MAX_ACCEL, 1.f);

/**
 * Slowdown Velocity for Waypoint
 *
 *
 * @unit m/s
 * @min 0.0
 * @max 100
 * @increment 0.001
 * @decimal 5
 * @group Rover Drive Control
 */
PARAM_DEFINE_FLOAT(RDC_WP_VEL, 0.5f);

