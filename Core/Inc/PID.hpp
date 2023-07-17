// this is basically this: https://github.com/UWARG/ZeroPilot-SW/blob/devel/Autopilot/Inc/PID.hpp

#include <cmath>
#include <cstdint>

class PID{
private:
	float kp, kd, ki;
	float i_max;
	float integral;
	float pastValue[3];
	float min_output;
	float max_output;

public:
	//Initializes the PID object.
	PID(double _kp, double _ki, double _kd, double _i_max, double _min_output, double _max_output);

	//Performs a PID calculation.
	//Needs to be run at a REGULAR interval, otherwise undefined behaviour may occur.
	double calculate(double desired, double actual, double actualRate);
};
