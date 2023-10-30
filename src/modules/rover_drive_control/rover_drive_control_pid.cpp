#include "rover_drive_control_pid.hpp"

float rover_drive_control_pid::pid(float reference, float actual, float dt, float windup, bool normalized, float kp, float ki, float kd)
{
	_reference = reference;
	_actual = actual;
	_dt = dt;
	_antireset_windup = windup;

	_kp = kp;
	_ki = ki;
	_kd = kd;

	if(normalized){
		_error = reference;
	} else {
		_error = (_reference - _actual);
	}

	// PX4_ERR("Anti windup %f", (double)_dt);

	if(abs(_error) > 1000){
		_error = 0;
	}

	_p_error = _error;

	if(!PX4_ISFINITE(_p_error)){
		_p_error = 0;
	}

	if((_i_error < _antireset_windup)){
		_i_error += _error*_dt;
		// PX4_ERR("Anti windup %f", (double)_p_error);
	}

	if(!PX4_ISFINITE(_i_error)){
		_i_error = 0;
	}

	_d_error = (_previous_error - _error)/dt;

	if(!PX4_ISFINITE(_d_error)){
		_d_error = 0;
	}

	// PX4_ERR("kp %f, ki %f, kd %f", (double)(_kp*_p_error), (double)(_ki*_i_error), (double)(_kd*_d_error));

	return _kp*_p_error + _ki*_i_error + _kd*_d_error;

}
