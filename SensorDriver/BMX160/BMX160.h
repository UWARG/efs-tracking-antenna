#ifndef INC_BMX160_HPP_
#define INC_BMX160_HPP_

#include "main.h"
#include "i2c.h"

/*----hardware indirect addressing-----*/
#define BMX160_DEVICE_ID   (0B1101000 << 1)
/*I2C driver address is left aligned
  LSB is reserved for R/W operation*/


#define MAG_IF_0 0X4C
/*
MAG_IF_0 {
  <7> mag_manual_enable
  <6> reserved
  <5:2> mag_offset
  <1:0> mag_rd_burst
}
*/
#define MAG_IF_1 0X4D //mag read addr
#define MAG_IF_2 0X4E //mag write addr
#define MAG_IF_3 0x4F //mag write data

//time increment 39 micro secs
#define SENSORTIME_0 0X18 //<7:0> sensor time
#define SENSORTIME_1 0X19 //<15:8> sensor time
#define SENSORTIME_2 0X1A //23:16> sensor time

#define MAG_X_0 0X04 //<7:0>
#define MAG_X_1 0X05 //<15:8>

#define MAG_Y_0 0X06 //<7:0>
#define MAG_Y_1 0X07 //<15:8>

#define MAG_Z_0 0X08 //<7:0>
#define MAG_Z_1 0X09 //<15:0>

#define _NULL 0X00

typedef struct {
  uint32_t MAG_X;
  uint32_t MAG_Y;
  uint32_t MAG_Z;
  uint32_t time_stamp;
} MAG_DATA;

class BMX160{
  public:
    BMX160(I2C_HandleTypeDef *hi2c);
    MAG_DATA Get_MAG_Data();
  private:
    uint32_t _32_BIT_APPEND(uint8_t _31_24, uint8_t _23_16, uint8_t _15_8, uint8_t _7_0) {
      return ((_31_24 << 24) | (_23_16 << 16) | (_15_8 << 8) | (_7_0));
    }

    void Mem_Write(uint16_t mem_addr, uint16_t mem_size, uint8_t *p_data, uint16_t size) {
      HAL_I2C_Mem_Write(HI2C, BMX160_DEVICE_ID, mem_addr, mem_size, p_data, size, (uint32_t)300);
    }

    void Mem_Read(uint16_t mem_addr, uint16_t mem_size, uint8_t *p_data, uint16_t size) {
      HAL_I2C_Mem_Read(HI2C, BMX160_DEVICE_ID, mem_addr, mem_size, p_data, size, (uint32_t)300);
      //see if possible add a macro to auto fetch mem_size and data_size
    }

    I2C_HandleTypeDef *HI2C;
};


#endif /* INC_BMX160_HPP_ */