#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include <stdint.h>


/**
 * @brief Configure and initialize the BMP280 sensor
 *
 * @param mode    Operating mode (BMP280_MODE_SLEEP/FORCED/NORMAL)
 *                Check page no.15 of datasheet
 * @param osrs_t  Temperature oversampling (BMP280_OSRS_1 ... 16 or SKIP)
 *                Check page no. 12 of datasheet
 * @param osrs_p  Pressure oversampling (BMP280_OSRS_1 ... 16 or SKIP)
 *                Check page no. 13 of datasheet
 * @param t_sb    Standby time in Normal mode (BMP280_T_SB_*)
 *                Check page no.16 of datasheet
 * @param filter  IIR filter setting (BMP280_FILTER_OFF ... _16)
 *                Check page no.13 of datasheet
 * @return 0 on success, negative error code on failure:
 *         -1 to -2: I2C errors
 *         -3: Wrong chip ID
 *         -4: Calibration read failed
 *         -5 to -6: Register write failed
 */
int BMP280_Config(uint8_t mode, uint8_t osrs_t, uint8_t osrs_p, uint8_t t_sb, uint8_t filter);

/**
 * @brief Read temperature and pressure
 *
 * @param temp_c    Pointer to store temperature in °C
 * @param press_hpa Pointer to store pressure in hPa (hectopascals)
 *
 * @return 0 on success, negative on error
 *
 * @note In Forced mode, call this function to trigger a single measurement.
 *       In Normal mode, it waits for measurement completion.
 */
int BMP280_ReadSensor(float *temp_c, float *press_hpa);


/* HAL expects LEFT-SHIFTED I2C address */
#define BMP280_I2C_ADDR                (0x77 << 1) // Change to (0x76 << 1) if SDO pin is low

#define SUPPORT_64BIT 1
//#define SUPPORT_32BIT 1

/* Registers */
#define BMP280_REG_CHIPID               0xD0
#define BMP280_REG_RESET                0xE0
#define BMP280_REG_STATUS               0xF3
#define BMP280_REG_CTRL_MEAS            0xF4
#define BMP280_REG_CONFIG               0xF5
#define BMP280_REG_PRESS_MSB            0xF7
#define BMP280_REG_TEMP_MSB             0xFA

/* Constants */
#define BMP280_CHIP_ID                  0x58
#define BMP280_RESET_VALUE              0xB6

/* Modes */
#define BMP280_MODE_SLEEP               0x00
#define BMP280_MODE_FORCED              0x01
#define BMP280_MODE_NORMAL              0x03

/* Oversampling */
#define BMP280_OSRS_SKIP                0x00
#define BMP280_OSRS_1                   0x01
#define BMP280_OSRS_2                   0x02
#define BMP280_OSRS_4                   0x03
#define BMP280_OSRS_8                   0x04
#define BMP280_OSRS_16                  0x05

/* Standby time (Normal mode only) */
#define BMP280_T_SB_0_5                 0x00 // 0.5  ms
#define BMP280_T_SB_62_5                0x01 // 62.5 ms
#define BMP280_T_SB_125                 0x02 // 125  ms
#define BMP280_T_SB_250                 0x03 // 250  ms
#define BMP280_T_SB_500                 0x04 // 500  ms
#define BMP280_T_SB_1000                0x05 // 1s
#define BMP280_T_SB_2000                0x06 // 2s
#define BMP280_T_SB_4000                0x07 // 4s

/* Filter coefficient */
#define BMP280_FILTER_OFF               0x00
#define BMP280_FILTER_2                 0x01
#define BMP280_FILTER_4                 0x02
#define BMP280_FILTER_8                 0x03
#define BMP280_FILTER_16                0x04


#endif /* INC_BMP280_H_ */
