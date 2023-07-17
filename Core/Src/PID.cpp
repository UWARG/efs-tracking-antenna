#include "PID.hpp"

PID::PID(double _kp, double _ki, double _kd, double _i_max, double _min_output, double _max_output){
	kp = _kp;
	kd = _kd;
	ki = _ki;

	i_max = _i_max;
	min_output = _min_output;
	max_output = _max_output;

	integral = 0.0f;
	pastValue[0] = 0.0f;
	pastValue[1] = 0.0f;
	pastValue[2] = 0.0f;
}

double PID::calculate(double desired, double actual, double actualRate) {

	double error = desired - actual;
	double derivative;

	integral +=  error;

	//Bounds for the integral variable
	if (integral < -i_max)
	{
		integral = -i_max;
	}
	else if (integral > i_max)
	{
		integral = i_max;
	}

	//If we are provided with a measured derivative (say from a gyroscope), it is always less noisy to use that than to compute it ourselves.
	if ( ! std::isnan(actualRate))
	{
		derivative = actualRate;
	}
	else
	{
		pastValue[2] = pastValue[1];
		pastValue[1] = pastValue[0];
		pastValue[0] = actual;

		// Finite difference approximation gets rid of noise much better than first order derivative computation
		derivative = ((3 * pastValue[0]) - (4 * pastValue[1]) + (pastValue[2]));
	}

	double ret = ((kp * error) + (ki * integral) - (kd * derivative));

	if (ret < min_output)
	{
		ret = min_output;
	}
	else if (ret > max_output)
	{
		ret = max_output;
	}

	return ret;
}
