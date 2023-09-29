#include <Wire.h>
#include <stdint.h>

void I2C_RxHandler(int numBytes) {
  uint32_t RxByte;
  while(Wire.available()) {  // Read Any Received Data
    RxByte = Wire.read(); 
  }
  Serial.print("Data received: ");
  Serial.println(RxByte, HEX);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(0X4C| 0X4D| 0X4E| 0X4F);//accept multiple address
  Wire.onReceive(I2C_RxHandler);//once the interrupt is triggered, call the function
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
