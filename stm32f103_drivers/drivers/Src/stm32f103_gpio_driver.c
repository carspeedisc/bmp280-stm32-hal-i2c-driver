/*
 * stm32f103_gpio_driver.c
 *
 *  Created on: Jul 22, 2024
 *      Author: Rajiv
 */
#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"

/******************************************************************
 * @fn                 -GPIO_PeripheralClockControl
 *
 * @description        -This function enables or disables the clock of gpio
 *
 * @param1             -base address of gpio peripheral
 * @param2             -Enable or Disable value
 * @param3             -none
 *
 * @return             -none
 *
 * @note               -none
 */

/*
 * Peripheral Clock Control
 */
void GPIO_Clk_Control(GPIO_def_struct *pGPIO,uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		if(pGPIO == GPIOA){
			GPIOA_CLK_EN();
		}
		else if(pGPIO == GPIOB){
			GPIOB_CLK_EN();
				}
		else if(pGPIO == GPIOC){
			GPIOC_CLK_EN();
						}
		else if(pGPIO == GPIOD){
			GPIOD_CLK_EN();
						}
		else if(pGPIO == GPIOE){
			GPIOE_CLK_EN();
						}

	}
	if(ENorDI == DISABLE){
		if(pGPIO == GPIOA){
			GPIOA_CLK_DI();
				}
		else if(pGPIO == GPIOB){
			GPIOB_CLK_DI();
				}
		else if(pGPIO == GPIOC){
			GPIOC_CLK_DI();
								}
		else if(pGPIO == GPIOD){
			GPIOD_CLK_DI();
							}
		else if(pGPIO == GPIOE){
			GPIOE_CLK_DI();

	}

}

/*
 * Init and DeInit
 */


	void GPIO_Init(GPIO_Handle_s *pGPIOx_Handle)
{
  uint32_t temp = 0;
  uint8_t temp1 = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber)/8;
  uint8_t temp2 = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber)%8;

	if(pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_OUT){

        temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinMode << (2*temp2));


        	pGPIOx_Handle->pGPIOx->CR[temp1] |= ~ (0xF*(4*(temp2)));
		    pGPIOx_Handle->pGPIOx->CR[temp1] |= temp;

        }

	else{
		/*
		 * will do later
		 */
	}
	temp = 0;
    temp = (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinPuPdControl << (pGPIOx_Handle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOx_Handle->pGPIOx->ODR |= temp;

  	temp = 0;
    temp = ((4*temp2)<< pGPIOx_Handle->GPIO_PinConfig.GPIO_PinOPType);
    pGPIOx_Handle->pGPIOx->CR[temp1] |= temp;

    temp = 0;
    temp = ((2*temp2) << pGPIOx_Handle->GPIO_PinConfig.GPIO_PinSpeed);
    pGPIOx_Handle->pGPIOx->CR[temp1] |= temp;


	}
}


void GPIO_DeInit(GPIO_def_struct *pGPIOx)
{

			if(pGPIOx == GPIOA){
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB){
				GPIOB_REG_RESET();
					}
			else if(pGPIOx == GPIOC){
				GPIOC_REG_RESET();
							}
			else if(pGPIOx == GPIOD){
				GPIOD_REG_RESET();
							}
			else if(pGPIOx == GPIOE){
				GPIOE_REG_RESET();

}

/*
 * Read and write from a Port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_Handle_s *pGPIOx_Handle)
{
	uint16_t value;
	value = ((uint16_t)pGPIOx_Handle->pGPIOx->IDR);
	return value;
}

}
void GPIO_WriteToFromOuputPort(GPIO_def_struct *pGPIOx,uint16_t val)
{
        pGPIOx->ODR = val;
}
/*
 * Read and write from a Pin
 */
uint8_t GPIO_ReadFromInputPin(GPIO_def_struct *pGPIOx,uint8_t GPIO_PinNumber)
{
	uint8_t value;
	value = (uint8_t)(pGPIOx->IDR &= (GPIO_PinNumber << 1));
	return value;

}
void GPIO_WriteToFromOuputPin(GPIO_def_struct *pGPIOx,uint8_t GPIO_PinNumber, uint8_t value)
{
 if(value == GPIO_PIN_SET){
	 pGPIOx->ODR |= (GPIO_PinNumber << 1);
 }
 else if(value == GPIO_PIN_RESET){
	 pGPIOx->ODR &= ~(GPIO_PinNumber << 1);
 }
}
/*
 * Toggling a Pin
 */
void GPIO_ToggleOutputPin(GPIO_def_struct *pGPIOx,uint8_t GPIO_PinNumber)
{
	 pGPIOx->ODR ^= (GPIO_PinNumber << 1);
}
/*
 * GPIO Interrupt Configuring and Handling
 */
void GPIO_IRQConfig(uint8_t IRQ_NUM,uint8_t IRQ_Priority,uint8_t ENorDI)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

