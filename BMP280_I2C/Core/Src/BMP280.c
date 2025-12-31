/**
 * @file BMP280.c
 * @brief Bosch BMP280 Digital Pressure and Temperature Sensor Driver
 *
 * This driver provides configuration and compensated reading of temperature
 * and pressure from the BMP280 sensor using STM32 HAL I2C functions.
 *
 * Features:
 * - Full compensation using factory calibration data
 * - Supports all oversampling modes, filter settings, and operating modes
 * - Returns temperature in °C and pressure in hPa
 *
 * @note Designed with STM32 HAL. Uses hi2c1 (defined externally in main.h)
 */

#include "BMP280.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

/* Calibration coefficients (read from sensor NVM) */
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;

static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t dig_P6, dig_P7, dig_P8, dig_P9;

/* Shared fine temperature value for pressure compensation */
static int32_t t_fine;

/* ==================== Internal Functions ==================== */

/**
 * @brief Read factory calibration data from BMP280 NVM
 * @return 0 on success, -1 on I2C error
 */
static int BMP280_ReadTrim(void)
{
    uint8_t buf[24];

    if (HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR, 0x88, I2C_MEMADD_SIZE_8BIT,
                         buf, 24, 100) != HAL_OK)
    {
        return -1;
    }

    /* Temperature coefficients */
    dig_T1 = (uint16_t)(buf[1] << 8) | buf[0];
    dig_T2 = (int16_t)(buf[3] << 8) | buf[2];
    dig_T3 = (int16_t)(buf[5] << 8) | buf[4];

    /* Pressure coefficients */
    dig_P1 = (uint16_t)(buf[7] << 8) | buf[6];
    dig_P2 = (int16_t)(buf[9] << 8) | buf[8];
    dig_P3 = (int16_t)(buf[11] << 8) | buf[10];
    dig_P4 = (int16_t)(buf[13] << 8) | buf[12];
    dig_P5 = (int16_t)(buf[15] << 8) | buf[14];
    dig_P6 = (int16_t)(buf[17] << 8) | buf[16];
    dig_P7 = (int16_t)(buf[19] << 8) | buf[18];
    dig_P8 = (int16_t)(buf[21] << 8) | buf[20];
    dig_P9 = (int16_t)(buf[23] << 8) | buf[22];

    return 0;
}

/**
 * @brief Compensate raw temperature reading
 * @param adc_T Raw ADC value from sensor
 * @return Temperature in 0.01°C units (e.g., 2517 = 25.17°C)
 */
// Global fine temperature value (used by both T and P compensation)
static int32_t t_fine;

// =============================================
// Temperature compensation (same for 32-bit and 64-bit versions)
// Returns temperature in °C as signed 32-bit integer Q8.8 format
// Output of 5120 = 20.00 °C
// =============================================
static int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8; // result in 0.01 °C units (Q8.8)

    return T;
}

// =============================================
// Pressure compensation - 64-bit version (recommended when possible)
// Returns pressure in Pa as uint32_t in Q24.8 format
// 24674848 = 24674848/256 ≈ 96386.2 Pa = 963.862 hPa
// =============================================
#if SUPPORT_64BIT

static uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);

    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)dig_P1) >> 33;

    if (var1 == 0)
        return 0; // avoid division by zero

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

    return (uint32_t)p;
}

#else // SUPPORT_32BIT version

// =============================================
// Pressure compensation - 32-bit only version
// Returns pressure in Pa as uint32_t (integer Pa)
// 96386 = 96386 Pa = 963.86 hPa
// =============================================
static uint32_t bmp280_compensate_P_int32(int32_t adc_P)
{
    int32_t var1, var2;
    uint32_t p;

    var1 = (((int32_t)t_fine >> 1) - (int32_t)64000);
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
    var2 = var2 + ((var1 * (int32_t)dig_P5) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);

    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((((int32_t)dig_P2) * var1) >> 1)) >> 18;

    var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);

    if (var1 == 0)
        return 0; // avoid division by zero

    p = (((uint32_t)((int32_t)1048576 - adc_P) - (var2 >> 12))) * 3125;

    if (p < 0x80000000UL)
        p = (p << 1) / (uint32_t)var1;
    else
        p = (p / (uint32_t)var1) * 2;

    var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;

    p = (uint32_t)((int32_t)p + ((var1 + var2 + (int32_t)dig_P7) >> 4));

    return p;
}

#endif
/* ==================== Public API ==================== */

/**
 * @brief Configure and initialize the BMP280 sensor
 *
 * @param mode Operating mode (BMP280_MODE_SLEEP/FORCED/NORMAL)
 * @param osrs_t Temperature oversampling (BMP280_OSRS_1 ... 16 or SKIP)
 * @param osrs_p Pressure oversampling (BMP280_OSRS_1 ... 16 or SKIP)
 * @param t_sb Standby time in Normal mode (BMP280_T_SB_*)
 * @param filter IIR filter setting (BMP280_FILTER_OFF ... _16)
 *
 * @return 0 on success, negative error code on failure:
 * -1 to -2: I2C errors
 * -3: Wrong chip ID
 * -4: Calibration read failed
 * -5 to -6: Register write failed
 */
int BMP280_Config(uint8_t mode, uint8_t osrs_t, uint8_t osrs_p, uint8_t t_sb, uint8_t filter)
{
    uint8_t tmp;

    /* Soft reset */
    tmp = BMP280_RESET_VALUE;
    if (HAL_I2C_Mem_Write(&hi2c1, BMP280_I2C_ADDR, BMP280_REG_RESET, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 100) != HAL_OK)
    {
        return -1;
    }

    HAL_Delay(20); // Datasheet recommends ~2ms, 20ms is safe

    /* Verify chip ID */
    if (HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR, BMP280_REG_CHIPID, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 100) != HAL_OK)
    {
        return -2;
    }

    if (tmp != BMP280_CHIP_ID)
    {
        return -3; // Not a BMP280
    }

    /* Read trim parameters */
    if (BMP280_ReadTrim() != 0)
    {
        return -4;
    }

    /* ctrl_meas: temp oversampling, pressure oversampling, mode */
    tmp = (osrs_t << 5) | (osrs_p << 2) | mode;
    if (HAL_I2C_Mem_Write(&hi2c1, BMP280_I2C_ADDR, BMP280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 100) != HAL_OK)
    {
        return -5;
    }

    /* config: standby time, filter, SPI disable */
    tmp = (t_sb << 5) | (filter << 2) | 0x00;
    if (HAL_I2C_Mem_Write(&hi2c1, BMP280_I2C_ADDR, BMP280_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 100) != HAL_OK)
    {
        return -6;
    }

    return 0;
}

/**
 * @brief Read compensated temperature and pressure
 *
 * @param temp_c Pointer to store temperature in °C
 * @param press_hpa Pointer to store pressure in hPa (hectopascals)
 *
 * @return 0 on success, negative on error
 *
 * @note In Forced mode, call this function to trigger a single measurement.
 * In Normal mode, it waits for measurement completion.
 */
int BMP280_ReadSensor(float *temp_c, float *press_hpa)
{
    uint8_t buf[6];
    int32_t adc_P, adc_T;

    /* Wait for measurement to complete (only relevant in Normal mode) */
    uint8_t status;
    do
    {
        if (HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR, BMP280_REG_STATUS, I2C_MEMADD_SIZE_8BIT, &status, 1, 100) != HAL_OK)
        {
            return -1;
        }
    } while (status & 0x08); // Bit 3: measuring

    /* Read burst: pressure (3 bytes) + temperature (3 bytes) */
    if (HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR, BMP280_REG_PRESS_MSB, I2C_MEMADD_SIZE_8BIT, buf, 6, 100) != HAL_OK)
    {
        return -2;
    }

    /* Reconstruct 20-bit raw values */
    adc_P = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
    adc_T = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));

    /* Compensate and convert to float */
    int32_t temp_raw = bmp280_compensate_T_int32(adc_T);
#if SUPPORT_64BIT
    uint32_t press_raw = bmp280_compensate_P_int64(adc_P);
    *press_hpa = press_raw / 256.0f; // Q24.8 to hPa
#else
    uint32_t press_raw = bmp280_compensate_P_int32(adc_P);
    *press_hpa = press_raw / 100.0f; // Integer Pa to hPa
#endif
    *temp_c = temp_raw / 100.0f; // Q8.8 to °C

    return 0;
}
