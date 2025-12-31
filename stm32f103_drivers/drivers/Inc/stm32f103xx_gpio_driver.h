
#include "stm32f103xx.h"
#include <stdint.h>
typedef struct
{
	uint8_t GPIO_PinNumber;      /* possible values from @GPIO_PIN_NO */
	uint8_t GPIO_PinMode;        /* possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;       /* possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl; /* possible values from @GPIO_IN_TYPE */
	uint8_t GPIO_PinOPType;      /* possible values from @GPIO_OP_TYPE */
	uint8_t GPIO_PinAltFunMode;

}GPIOx_PinConfig_s;

typedef struct
{
	GPIO_def_struct *pGPIOx;
	GPIOx_PinConfig_s GPIO_PinConfig;
}GPIO_Handle_s;

/*
 * Peripheral Clock Control
 */
void GPIO_Clk_Control(GPIO_def_struct *pGPIO,uint8_t ENorDI);

/*
 * Init and DeInit
 */

void GPIO_Init(GPIO_Handle_s *pGPIOx_Handle);
void GPIO_DeInit(GPIO_def_struct *pGPIOx);

/*
 * Read and write from a Port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_Handle_s *pGPIOx_Handle);
void GPIO_WriteToFromOuputPort(GPIO_def_struct *pGPIOx,uint16_t val);

/*
 * Read and write from a Pin
 */
uint8_t GPIO_ReadFromInputPin(GPIO_def_struct *pGPIOx,uint8_t GPIO_PinNumber);
void GPIO_WriteToFromOuputPin(GPIO_def_struct *pGPIOx,uint8_t GPIO_PinNumber, uint8_t value);

/*
 * Toggling a Pin
 */
void GPIO_ToggleOutputPin(GPIO_def_struct *pGPIOx,uint8_t GPIO_PinNumber);

/*
 * GPIO Interrupt Configuring and Handling
 */
void GPIO_IRQConfig(uint8_t IRQ_NUM,uint8_t IRQ_Priority,uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);

/*@GPIO_PIN_NO
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0              0
#define GPIO_PIN_NO_1              1
#define GPIO_PIN_NO_2              2
#define GPIO_PIN_NO_3              3
#define GPIO_PIN_NO_4              4
#define GPIO_PIN_NO_5              5
#define GPIO_PIN_NO_6              6
#define GPIO_PIN_NO_7              7
#define GPIO_PIN_NO_8              8
#define GPIO_PIN_NO_9              9
#define GPIO_PIN_NO_10             10
#define GPIO_PIN_NO_11             11
#define GPIO_PIN_NO_12             12
#define GPIO_PIN_NO_13             13
#define GPIO_PIN_NO_14             14
#define GPIO_PIN_NO_15             15
#define GPIO_PIN_NO_16             16

/* @GPI_PIN_MODES
 * GPIO pins modes
 */
#define GPIO_MODE_IN               0
#define GPIO_MODE_OUT              1
#define GPIO_MODE_ANALOG           0
#define GPIO_MODE_ALTFUN           1
#define GPIO_MODE_IT_RT            4
#define GPIO_MODE_IT_FT            5
#define GPIO_MODE_IT_RTFT          6

/* @GPIO_OP_TYPE
 * GPIO pins output types
 */
#define GPIO_OP_TYPE_PP            0
#define GPIO_OP_TYPE_OD            1

/* @GPIO_PIN_SPEED
 * GPIO pin operating speeds
 */
#define GPIO_SPEED_10MHz           1
#define GPIO_SPEED_2MHz            2
#define GPIO_SPEED_50MHz           3

/* @GPIO_IN_TYPE
 * GPIO pin input types
 */
#define GPIO_FLOATING              1
#define GPIO_PIN_PU                1
#define GPIO_PIN_PD                0





















