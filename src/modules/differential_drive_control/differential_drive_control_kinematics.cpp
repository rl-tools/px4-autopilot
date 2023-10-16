#include "differential_drive_control_kinematics.hpp"


void differential_drive_control_kinematics::setInput(const matrix::Vector2f& input)
{
    	_input = input;

	_vx = _input(0);
	_omega = _input(1);
}

void differential_drive_control_kinematics::computeControl()
{
	float r = _param_ddc_wheel_radius.get();
	float l = _param_ddc_wheel_base.get();

	_motor_vel_right = _vx/r + l/2 * _omega/r;
	_motor_vel_left = _vx/r - l/2 * _omega/r;

	_output(0) = _motor_vel_right;
	_output(1) = _motor_vel_left;
}

matrix::Vector2f differential_drive_control_kinematics::getOutput() const
{
    	return _output;
}
