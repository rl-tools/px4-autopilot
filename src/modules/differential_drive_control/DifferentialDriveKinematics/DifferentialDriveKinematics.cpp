#include "DifferentialDriveKinematics.hpp"

void DifferentialDriveKinematics::setWheelBase(float wheel_base)
{
	if (wheel_base <= 0) {
		printf("Wheel base must be greater than 0\n");
		return;
	}

	_wheel_base = wheel_base;
}

void DifferentialDriveKinematics::setWheelRadius(float wheel_radius)
{
	if (wheel_radius <= 0) {
		printf("Wheel radius must be greater than 0\n");
		return;
	}

	_wheel_radius = wheel_radius;
}

matrix::Vector2f DifferentialDriveKinematics::computeInverseKinematics(float linear_vel_x, float yaw_rate)
{
	float motor_vel_right = linear_vel_x / _wheel_radius - _wheel_base / 2 * yaw_rate / _wheel_radius;
	float motor_vel_left = linear_vel_x / _wheel_radius + _wheel_base / 2 * yaw_rate / _wheel_radius;

	return matrix::Vector2f(motor_vel_right, motor_vel_left);
}
