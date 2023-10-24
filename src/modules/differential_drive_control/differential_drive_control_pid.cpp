#include "differential_drive_control_pid.hpp"

float differential_drive_control_pid::pid(float reference, float actual, float dt, float windup, bool normalized)
{
	_reference = reference;
	_actual = actual;
	_dt = dt;
	_antireset_windup = windup;

	if(normalized){
		_p_error = reference;
	} else {
		_p_error = (_reference - _actual)*_dt;
	}

	if(!PX4_ISFINITE(_p_error)){
		_p_error = 0;
	}

	if((_i_error < _antireset_windup) && (_antireset_windup > 0)){
		_i_error += _p_error*_dt;
	}

	if(!PX4_ISFINITE(_i_error)){
		_i_error = 0;
	}

	_d_error = (_previous_error - _p_error)/dt;

	if(!PX4_ISFINITE(_d_error)){
		_d_error = 0;
	}

	return _kp*_p_error + _ki*_i_error + _kd*_d_error;

}
