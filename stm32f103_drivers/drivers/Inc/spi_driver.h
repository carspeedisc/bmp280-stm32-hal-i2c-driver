/*
 * spi_derivers.h
 *
 *  Created on: Jul 30, 2024
 *      Author: Rajiv
 */

#include "stm32f103xx.h"

typedef struct{

	uint8_t Device_mode;
	uint8_t Bus_config;
	uint8_t DFF;
	uint8_t Sclk_speed;
	uint8_t CPOL;
	uint8_t CPHA;
	uint8_t SSM;


}SPI_PinConfig_s;

typedef struct {

	SPI_Regdef *pSPIx;
	SPI_PinConfig_s SPI_Config;
}SPI_Handle_s;

/*
 * SPI Peripheral Clock Control
 */
void SPI_CLK_CONTROL(SPI_Regdef *pSPIx, uint8_t ENorDI);
/*
 * SPI Peripheral Init and DeInit
 */
void SPI_Init(SPI_Handle_s *pSPI_handle);
void SPI_DeInit(SPI_Regdef *pSPIx);

/*
 * SPI Send and Receive data
 */
void SPI_ReceiveData(SPI_Handle_s *pSPI_handle, uint8_t Rxbuffer, uint32_t Len);
void SPI_SendData(SPI_Handle_s *pSPI_handle, uint8_t Rxbuffer, uint32_t Len);

/*
 * SPI Interrupt Configuring and Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQ_NUM,uint8_t IRQ_Priority,uint8_t ENorDI);
void SPI_IRQPriority(uint8_t IRQ_NUM,uint8_t IRQ_Priority,uint8_t ENorDI);
void SPI_IRQHandling(SPI_Handle_s *pSPI_handle);







