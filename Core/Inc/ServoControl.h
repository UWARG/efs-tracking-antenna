#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

namespace Servo{
	/*direction of motor movement*/
	enum TurningDirection {
		clock_wise = 0,
		counter_clock_wise
	};

	/*store the information after motor calibration*/
	struct RotationStatus {
		uint8_t revolve_count;
		TurningDirection direction;
	};
	typedef RotateStatus* MotorStatus;

	/*store the latest sensor data from bmx160*/
	struct SensorOut {
		BMX_Struct* data;
		uint8_t is_new;
	};
	typedef SensorOut* SensorData;

	class MotorControl{
		public:
			MotorControl(

			calibration();
			
			get_current_angle();

			motor_movement();
		private:

			uint16_t init_angle;
			MotorStatus motor_status;
			SensorData sensor_output;
	}
}


#endif //SERVOCONTROL_H
