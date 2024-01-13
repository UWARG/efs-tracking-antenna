#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DPEng_BMX160.h>
#include <Mahony_BMX160.h>
#include <Madgwick_BMX160.h>

DPEng_BMX160 dpEng = DPEng_BMX160(0x160A, 0x160B, 0x160C);
Mahony_BMX160 filter;

// Offsets applied to raw x/y/z mag values
const float mag_offsets[3] = {20.58,  -29.25,  -18.64};

// Soft iron error compensation matrix
const float soft_iron[3][3] = {
  { 0.616,  0.007, 0.020 },
  { 0.007,  0.607, -0.027 },
  { 0.020, -0.027,  2.627 }
};

float mag_data[3];

float mag_field_strength = 60.19F; // Not used.
float mag_decl = -9.466667;

float q0,q1,q2,q3; // Quaternion values
float rotationMatrix[3][3];

void setup()
{
  Serial.begin(115200);

  if(!dpEng.begin(BMX160_ACCELRANGE_4G, GYRO_RANGE_250DPS))
  {
    Serial.println("Ooops, no sensor detected ... Check your wiring!");
    while(1);
  }

  filter.begin();
}

void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t mag_event; 

  dpEng.getEvent(&accel_event, &gyro_event, &mag_event);

  mag_data[0]= mag_event.magnetic.x - mag_offsets[0];
  mag_data[1]= mag_event.magnetic.y - mag_offsets[1];
  mag_data[2]= mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  for (uint8_t i = 0; i < 3; i++) {
      mag_data[i] = 
                    (soft_iron[i][0] * mag_data[0]) +
                    (soft_iron[i][1] * mag_data[1]) +
                    (soft_iron[i][2] * mag_data[2]);
  }

  filter.update(gyro_event.gyro.x, gyro_event.gyro.y, gyro_event.gyro.z, // GYRO DATA
              accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z, // ACCEL DATA
              mag_data[0], mag_data[1], mag_data[2], // MAG DATA
              mag_event.timestamp);
      
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw() + mag_decl;



  Serial.print(millis());
  Serial.print(" - Orientation: ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);

  delay(10);
}