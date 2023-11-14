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
	_motor_vel_right = _linear_vel_x/_wheel_radius + _wheel_base/2 * _yaw_rate/_wheel_radius;
	_motor_vel_left = _linear_vel_x/_wheel_radius - _wheel_base/2 * _yaw_rate/_wheel_radius;

	_output_inverse(0) = _motor_vel_right;
	_output_inverse(1) = _motor_vel_left;
}

void DifferentialDriveKinematics::computeForwardsKinematics()
{
	_linear_vel_x = _wheel_radius/2*(_motor_vel_right + _motor_vel_left);
	_yaw_rate = _wheel_radius/_wheel_base*(_motor_vel_right - _motor_vel_left);

	_output_forwards(0) = _linear_vel_x;
	_output_forwards(1) = _yaw_rate;
}

void DifferentialDriveKinematics::setWheelBase(float wheel_base)
{
	if(wheel_base <= 0){
		printf("Wheel base must be greater than 0\n");
		return;
	}

	_wheel_base = wheel_base;
}

void DifferentialDriveKinematics::setWheelRadius(float wheel_radius)
{
	if(wheel_radius <= 0){
		printf("Wheel radius must be greater than 0\n");
		return;
	}

	_wheel_radius = wheel_radius;
}

matrix::Vector2f DifferentialDriveKinematics::getOutput(bool inverse) const
{
	if(inverse){
		return _output_inverse;
	} else {
		return _output_forwards;
	}

}
