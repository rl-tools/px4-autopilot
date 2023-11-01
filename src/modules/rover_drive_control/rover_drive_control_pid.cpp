#include "rover_drive_control_pid.hpp"

float rover_drive_control_pid::pid(float reference, float actual, float dt, float windup, float saturation, bool useReferenceAsError, float kp, float ki, float kd)
{
	_reference = reference;
	_actual = actual;
	_dt = dt;
	_antireset_windup = windup;

	_kp = kp;
	_ki = ki;
	_kd = kd;

	_error = useReferenceAsError ? reference : (reference - actual);

	if(!PX4_ISFINITE(_error)){
		_error = 0;
	}

	_p_error = _error;

	if((_i_error < _antireset_windup) && _previous_output < saturation){
		_i_error += _error*_dt;
	} else {
		_i_error += 0;
	}

	if(!PX4_ISFINITE(_i_error)){
		_i_error = 0;
	}

	_d_error = (_error - _previous_error)/dt;

	if(!PX4_ISFINITE(_d_error)){
		_d_error = 0;
	}

	float output = _kp*_p_error + _ki*_i_error + _kd*_d_error;

	_previous_output = output;

	return math::constrain(output, -saturation, saturation);
}
