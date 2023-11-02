#ifndef INC_BMX160_H_
#define INC_BMX160_H_

#include "main.h"
#include "i2c.h"
#include <cstdint>

/*----hardware indirect addressing-----*/
#define BMX160_DEVICE_ID   (0B01101000 << 1)
/*I2C driver address is left aligned
  LSB is reserved for R/W operation*/

#define MAG_RESOLUTION 0.3f
/*the LSB stands for least significant bit change => normally if using ADC increment by 1 implies 0.3 actual unit change
 * https://www.azosensors.com/article.aspx?ArticleID=267
 */

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

#define RHALL_0 0X0A //<7:0>
#define RHALL_1 0X0B //<15:8>

#define CHIP_ID_ADDR 0X00

#define BMX_MEM_SIZE sizeof(uint8_t)

typedef struct {
  double MAG_X;
  double MAG_Y;
  double MAG_Z;
  uint32_t time_stamp;
  uint32_t R_hall;//hall effect sensor output
  bool data_invalid;//Note: based on the data sheet the value should be +-1300 for and x,y; +-2500 for z
} MAG_DATA;
/*
 *Calculation to get pT = 16-bit sensor data * MAG_RESOLUTION
 */

#define	E_NOT_OK 0
#define	E_OK 1
#define Chip_Status int8_t
#define Sensor_Status bool
#define CHIP_ID 0XD8

class BMX160{
  public:
    BMX160(I2C_HandleTypeDef *hi2c) {
    	HI2C = hi2c;
    }

    MAG_DATA Get_MAG_Data();

    Sensor_Status Sensor_Available(uint8_t timeout_count);
  private:

    Chip_Status _Get_ChipID();

    uint32_t _32_BIT_APPEND(uint8_t _31_24, uint8_t _23_16, uint8_t _15_8, uint8_t _7_0) {
      return ((_31_24 << 24) | (_23_16 << 16) | (_15_8 << 8) | (_7_0));
    }

    HAL_StatusTypeDef Mem_Write(uint16_t mem_addr, uint8_t *p_data) {
      return HAL_I2C_Mem_Write(HI2C, BMX160_DEVICE_ID, mem_addr, BMX_MEM_SIZE, p_data, BMX_MEM_SIZE, (uint32_t)300);
    }

    HAL_StatusTypeDef Mem_Read(uint16_t mem_addr, uint8_t *p_data) {
      return HAL_I2C_Mem_Read(HI2C, BMX160_DEVICE_ID, mem_addr, BMX_MEM_SIZE, p_data, BMX_MEM_SIZE, (uint32_t)300);
      //see if possible add a macro to auto fetch mem_size and data_size
    }

    I2C_HandleTypeDef *HI2C;
};


#endif /* INC_BMX160_H_ */
