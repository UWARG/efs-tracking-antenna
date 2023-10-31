To-do:
the file structure for this project will be the following

- Core
  	-Inc
		- IMU public interface
		- GPS public interface
		- servo public interface(angle based)
		- Mavlink public interface
		- Tracking algorithm public interface

- Mavlink
	- mission planner simulator support(python)
	- mavlink gps decoder
- SensorDriver
	- BMX160 driver(will be called by IMU public interface)
	- NEO M8 driver(will be called by GPS public interace)
	- HS-785HB Driver
- Tracking antenna algorithm


Pin definition for the board:
- ADC_IN3: ADC input reserved for current sensing
	- PC2
- UART3: Mavlink communication(either to simulator or to airside)
	- PB10 TX
	- PB11 RX
- UART5: NEO_M8 UART port
	- PD2 RX
	- PC12 TX
- I2C2: BMX driver
	- PF0 SDA(need pull-up)
	- PF1 CLK(need pull-up)
- TIM1 CH1: Yaw PWM
	- PE9
- TIM1 CH2: Pitch PWM
	- PE11
