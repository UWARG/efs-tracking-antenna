#include <Wire.h>
#include <stdint.h>

const uint8_t slave_addr[4] = {0X4C, 0X4D, 0X4E, 0X4F};

uint8_t _Slave_Addr;

uint8_t update_slave_addr(){
  static uint8_t index_pointer = 0;
  if (index_pointer == 4) {
    index_pointer == 0;
  }
  _Slave_Addr = slave_addr[index_pointer];
  index_pointer += 1;
}

void I2C_RxHandler(int numBytes) {
  uint32_t RxByte;
  while(Wire.available()) {  // Read Any Received Data
    RxByte = Wire.read(); 
  }
  Serial.print("The received address is ");
  Serial.print(_Slave_Addr, HEX);
  Serial.print(", and the data received are ");
  Serial.println(RxByte, HEX);

  update_slave_addr();
  Wire.begin(_Slave_Addr);
  Wire.onReceive(I2C_RxHandler);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  update_slave_addr();
  Wire.begin(_Slave_Addr);
  Wire.onReceive(I2C_RxHandler);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
