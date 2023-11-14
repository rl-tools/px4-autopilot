#include "DifferentialDriveKinematics.hpp"


void DifferentialDriveKinematics::setInput(const matrix::Vector2f& input, bool inverse)
{
    	_input = input;

	if(inverse){
		_linear_vel_x = _input(0);
		_yaw_rate = _input(1);
		computeInverseKinematics();
	} else {
		_motor_vel_right = _input(0);
		_motor_vel_left = _input(1);
		computeForwardsKinematics();
	}

}

void DifferentialDriveKinematics::computeInverseKinematics()
{
	float r = _param_rdc_wheel_radius.get();
	float l = _param_rdc_wheel_base.get();

	_motor_vel_right = _linear_vel_x/r + l/2 * _yaw_rate/r;
	_motor_vel_left = _linear_vel_x/r - l/2 * _yaw_rate/r;

	_output_inverse(0) = _motor_vel_right;
	_output_inverse(1) = _motor_vel_left;
}

void DifferentialDriveKinematics::computeForwardsKinematics()
{
	float r = _param_rdc_wheel_radius.get();
	float l = _param_rdc_wheel_base.get();

	_linear_vel_x = r/2*(_motor_vel_right + _motor_vel_left);
	_yaw_rate = r/l*(_motor_vel_right - _motor_vel_left);

	_output_forwards(0) = _linear_vel_x;
	_output_forwards(1) = _yaw_rate;
}

matrix::Vector2f DifferentialDriveKinematics::getOutput(bool inverse) const
{
	if(inverse){
		return _output_inverse;
	} else {
		return _output_forwards;
	}

}
