#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include <stdint.h>

/* HAL expects LEFT-SHIFTED I2C address */
#define BMP280_I2C_ADDR (0x77 << 1) // Change to (0x76 << 1) if SDO pin is low

/* Registers */
#define BMP280_REG_CHIPID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB 0xFA

/* Constants */
#define BMP280_CHIP_ID 0x58
#define BMP280_RESET_VALUE 0xB6

/* Modes */
#define BMP280_MODE_SLEEP 0x00
#define BMP280_MODE_FORCED 0x01
#define BMP280_MODE_NORMAL 0x03

/* Oversampling */
#define BMP280_OSRS_SKIP 0x00
#define BMP280_OSRS_X1 0x01
#define BMP280_OSRS_X2 0x02
#define BMP280_OSRS_X4 0x03
#define BMP280_OSRS_X8 0x04
#define BMP280_OSRS_X16 0x05

/* Standby time (Normal mode only) */
#define BMP280_T_SB_0_5 0x00 // 0.5 ms
#define BMP280_T_SB_62_5 0x01
#define BMP280_T_SB_125 0x02
#define BMP280_T_SB_250 0x03
#define BMP280_T_SB_500 0x04
#define BMP280_T_SB_1000 0x05
#define BMP280_T_SB_2000 0x06
#define BMP280_T_SB_4000 0x07

/* Filter coefficient */
#define BMP280_FILTER_OFF 0x00
#define BMP280_FILTER_2 0x01
#define BMP280_FILTER_4 0x02
#define BMP280_FILTER_8 0x03
#define BMP280_FILTER_16 0x04

/* Public API */
int BMP280_Config(uint8_t mode, uint8_t osrs_t, uint8_t osrs_p,
                  uint8_t t_sb, uint8_t filter);

int BMP280_ReadSensor(float *temp_c, float *press_hpa);

#endif /* INC_BMP280_H_ */
