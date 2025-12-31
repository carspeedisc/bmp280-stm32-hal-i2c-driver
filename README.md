
# BMP280 STM32 HAL Driver

**STM32 HAL-based driver** for the **Bosch BMP280** digital barometric pressure and temperature sensor.

## Features

- Full integer-based compensation (64-bit preferred, 32-bit fallback)
- Uses **STM32 HAL I2C** (portable across STM32 families)
- **Integer-only** compensation algorithms (no floating point in core math)

## Requirements

- STM32CubeIDE / STM32CubeMX generated project
- HAL I2C driver enabled

## Hardware Connections

| BMP280 Pin | STM32 Pin     | Notes                                    |
|------------|---------------|------------------------------------------|
| VCC        | 3.3V          | 1.71–3.6V supported                      |
| GND        | GND           |                                          |
| SCL        | I2C SCL       | Pull-up required (4.7kΩ typical)         |
| SDA        | I2C SDA       | Pull-up required                         |
| SDO        | GND or 3.3V   | Selects I2C address (0x76 or 0x77)       |

**Default I2C address**: `0x77 << 1` (SDO = high)  
Change to `0x76 << 1` in `BMP280.h` if SDO is connected to GND.

## Installation

Copy the `BMP280_HAL` folder into your STM32CubeIDE project.

(or clone directly)

```bash
git clone https://github.com/carspeedisc/bmp280-stm32-hal-i2c-driver.git
```

## Quick Usage

Call this **once** after I2C is initialized:

```c
#include "BMP280_HAL.h"

int main(void)
{
    // ... HAL_Init(), SystemClock_Config(), MX_I2C1_Init() etc.

    // Recommended common configuration
    if (BMP280_Config(
            BMP280_MODE_NORMAL,     // Continuous automatic measurements
            BMP280_OSRS_2,          // Temperature ×2 oversampling
            BMP280_OSRS_8,          // Pressure ×8 (very good accuracy/noise balance)
            BMP280_T_SB_500,        // 500 ms standby - ~2 measurements/second
            BMP280_FILTER_4         // Moderate IIR filtering for smooth pressure
        ) != 0)
    {
        // Handle error (I2C fail, wrong chip ID, etc.)
        while(1);
    }

    while (1)
    {
        float temperature = 0.0f;
        float pressure = 0.0f;

        if (BMP280_ReadSensor(&temperature, &pressure) == 0)
        {
            // Use the values
            // printf("T: %.2f °C   P: %.1f hPa\n", temperature, pressure);
        }

        HAL_Delay(1000);
    }
}
```

### BMP280_Config – Detailed Parameter Explanation

```c
int BMP280_Config(uint8_t mode, uint8_t osrs_t, uint8_t osrs_p, uint8_t t_sb, uint8_t filter);
```

This function performs a full sensor initialization:
- Soft reset
- Chip ID verification (must be 0x58)
- Loading factory calibration data
- Writing all configuration registers

**Parameter overview**:

| Parameter  | Meaning                              | Most useful / recommended values                          | Impact / Notes                                                                 |
|------------|--------------------------------------|-----------------------------------------------------------|--------------------------------------------------------------------------------|
| `mode`     | Operating mode                       | `BMP280_MODE_NORMAL`<br>`BMP280_MODE_FORCED`<br>`BMP280_MODE_SLEEP` | Normal - continuous<br>Forced - one measurement then sleep<br>Sleep - lowest power |
| `osrs_t`   | Temperature oversampling             | `BMP280_OSRS_1`, `BMP280_OSRS_2`, `BMP280_OSRS_SKIP`     | ×1 or ×2 is usually enough (temperature changes slowly)                        |
| `osrs_p`   | Pressure oversampling                | `BMP280_OSRS_8`, `BMP280_OSRS_16`, `BMP280_OSRS_4`       | Higher = better resolution & lower noise (most important parameter for accuracy) |
| `t_sb`     | Standby time (Normal mode only)      | `BMP280_T_SB_500`, `BMP280_T_SB_1000`, `BMP280_T_SB_250` | Time between measurements - affects power consumption and sampling rate        |
| `filter`   | IIR filter coefficient for pressure  | `BMP280_FILTER_4`, `BMP280_FILTER_8`, `BMP280_FILTER_OFF` | Stronger filtering = smoother output but slower response to real pressure changes |

**Recommended starting point** (very good balance between accuracy, speed and power):

```c
BMP280_Config(BMP280_MODE_NORMAL, BMP280_OSRS_2, BMP280_OSRS_8, BMP280_T_SB_500, BMP280_FILTER_4);
```

### BMP280_ReadSensor

```c
int BMP280_ReadSensor(float *temp_c, float *press_hpa);
```

**Returns**:
- `0` - success (values updated)
- negative - error (I2C communication or measurement problem)


