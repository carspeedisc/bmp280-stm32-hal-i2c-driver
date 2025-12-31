/*
 * spi_driver.c
 *
 *  Created on: Aug 5, 2024
 *      Author: Rajiv
 */

#include "stm32f103xx.h"
#include <stdint.h>

/******************************************************************
 * @fn                 -SPI_PeripheralClockControl
 *
 * @description        -This function enables or disables the clock of SPI
 *
 * @param1             -base address of SPI peripheral
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
void SPI_Clk_Control(SPI_Regdef *pSPIx,uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
		if(pSPIx == SPI1){
			SPI1_CLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_CLK_EN();
				}
		else if(pSPIx == SPI3){
			SPI3_CLK_EN();
						}

	}
	if(ENorDI == DISABLE){
		if(pSPIx == SPI1){
			SPI1_CLK_DI();
				}
		else if(pSPIx == SPI2){
			SPI2_CLK_DI();
				}
		else if(pSPIx == SPI3){
			SPI3_CLK_DI();
								}

	}

}

/*
 * Init and DeInit
 */
void SPI_DeInit(SPI_Regdef *pSPIx)
{

			if(pSPIx == SPI1){
				SPI1_REG_RESET();
			}
			else if(pSPIx == SPI2){
				SPI2_REG_RESET();
					}
			else if(pSPIx == SPI3){
				SPI3_REG_RESET();
							}


}
