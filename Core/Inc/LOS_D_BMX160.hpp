#ifndef INC_BMX160_HPP_
#define INC_BMX160_HPP_

/*----hardware indirect addressing-----*/
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

#define SENSORTIME_0 0X18 //<7:0> sensor time
#define SENSORTIME_1 0X19 //<15:8> sensor time
#define SENSORTIME_2 0X1A //23:16> sensor time

#define MAG_X_0 0X04 //<7:0>
#define MAG_X_1 0X05 //<15:8>

#define MAG_Y_0 0X06 //<7:0>
#define MAG_Y_1 0X07 //<15:8>

#define MAG_Z_0 0X08 //<7:0>
#define MAG_Z_1 0X09 //<15:0>

#define NULL 0X00

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

    void I2C_Write(uint16_t addr, uint8_t *p_data) {
      HAL_I2C_Master_Transmit(HI2C, addr, p_data, 300);
    }

    void I2C_Read(uint16_t addr, uint8_t *p_data, uint16_t size) {
      HAL_I2C_Master_Recieve(HI2C, addr, p_data, size, 300);
    }

    I2C_HandleTypeDef *HI2C;
}


#endif /* INC_BMX160_HPP_ */
