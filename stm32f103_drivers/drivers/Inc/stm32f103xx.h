/*
 * bluepill.h
 *
 *  Created on: Jul 18, 2024
 *      Author: Rajiv
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdint.h>

#define FLASH_BASEADDR                0x08000000U
#define SRAM_BASEADDR                 0x20000000U
#define ROM_BASEADDR                  0x1FFFF000U


#define PERIPHERAL_BASEADDR           0x40000000U
#define AHB_BASEADDR                  0x40018000U
#define APB1PERIPH_BASEADDR           PERIPHERAL_BASEADDR
#define APB2PERIPH_BASEADDR           0x40010000U

#define RCC_BASEADDR                  0x40021000

/*BASE ADDRESSES ON APB1 BUS*/

/*Backup Register*/
#define BKP_BASEADDR                  (APB1PERIPH_BASEADDR + 0x6C00)

#define CAN1_BASEADDR                 (APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASEADDR                 (APB1PERIPH_BASEADDR + 0x6800)

#define DAC_BASEADDR                  (APB1PERIPH_BASEADDR + 0x7400)

#define IWDG_BASEADDR                 (APB1PERIPH_BASEADDR + 0x3000)

#define I2C1_BASEADDR                 (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                 (APB1PERIPH_BASEADDR + 0x5800)

#define I2S_BASEADDR                  (APB1PERIPH_BASEADDR + 0x3800)

/*Power Control*/
#define PWR_BASEADDR                  (APB1PERIPH_BASEADDR + 0x7000)

/*Real time clock*/
#define RTC_BASEADDR                  (APB1PERIPH_BASEADDR + 0x2800)

/*BASE ADDRESS of I2S is same as SPI2*/
#define SPI2_BASEADDR                 (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                 (APB1PERIPH_BASEADDR + 0x3C00)

/*USB AND CAN SHARED SRAM*/
#define SHARED_SRAM_BASEADDR          (APB1PERIPH_BASEADDR + 0x6000)

#define TIM2_BASEADDR                  APB1PERIPH_BASEADDR
#define TIM3_BASEADDR                 (APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR                 (APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR                 (APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR                 (APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR                 (APB1PERIPH_BASEADDR + 0x1400)
#define TIM12_BASEADDR                (APB1PERIPH_BASEADDR + 0x1800)
#define TIM13_BASEADDR                (APB1PERIPH_BASEADDR + 0x1C00)
#define TIM14_BASEADDR                (APB1PERIPH_BASEADDR + 0x2000)

#define USART2_BASEADDR               (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR               (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR                (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR                (APB1PERIPH_BASEADDR + 0x5000)

/*USB DEVICE FS REGISTER*/
#define USB_FS_REG_BASEADDR           (APB1PERIPH_BASEADDR + 0x5C00)

#define WWDG_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4000)


/*BASE ADDRESSES ON APB2 BUS*/

#define ADC1_BASEADDR                 (APB2PERIPH_BASEADDR + 0x2400)
#define ADC2_BASEADDR                 (APB2PERIPH_BASEADDR + 0x2800)
#define ADC3_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3C00)

#define AFIO_BASEADDR                  APB2PERIPH_BASEADDR

#define EXTI_BASEADDR                 (APB2PERIPH_BASEADDR + 0x0400)


#define GPIOA_BASEADDR                (APB2PERIPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR                (APB2PERIPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR                (APB2PERIPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR                (APB2PERIPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR                (APB2PERIPH_BASEADDR + 0x1800)
#define GPIOF_BASEADDR                (APB2PERIPH_BASEADDR + 0x1C00)
#define GPIOG_BASEADDR                (APB2PERIPH_BASEADDR + 0x2000)

#define SPI1_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3000)


#define TIM1_BASEADDR                 (APB2PERIPH_BASEADDR + 0x2C00)
#define TIM8_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3400)
#define TIM9_BASEADDR                 (APB2PERIPH_BASEADDR + 0x4C00)
#define TIM10_BASEADDR                (APB2PERIPH_BASEADDR + 0x5000)
#define TIM11_BASEADDR                (APB2PERIPH_BASEADDR + 0x5400)

#define USART1_BASEADDR               (APB2PERIPH_BASEADDR + 0x3800)


#define ENABLE              1
#define DISABLE             0
#define SET                 1
#define RESET               0
#define GPIO_PIN_SET        1
#define GPIO_PIN_RESET      0

/*************************************************Peripheral registers definition structures****************************************************/

/*_________________________________________________gpio registers definiton_________________________________________________*/

typedef struct{

	volatile uint32_t CR[2];
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;

}GPIO_def_struct;

typedef struct{

	volatile uint32_t RCC_CR;
	volatile uint32_t RCC_CFGR;
	volatile uint32_t RCC_CIR;
	 uint32_t RCC_APB2RSTR;
	 uint32_t RCC_APB1RSTR;
	volatile uint32_t RCC_AHBENR;
	 uint32_t RCC_APB2ENR;
	 uint32_t RCC_APB1ENR;
	volatile uint32_t RCC_BDCR;
	volatile uint32_t RCC_CSR;
	volatile uint32_t RCC_AHBSTR;
	volatile uint32_t RCC_CFGR2;
}RCC_Regdef;

typedef struct{

	uint32_t SPI_CR1;
	uint32_t SPI_CR2;
	uint32_t SPI_SR;
	uint32_t SPI_DR;
	uint32_t SPI_CRCPR;
	uint32_t SPI_RXCRCR;
	uint32_t SPI_TXCRCR;
	uint32_t SPI_I2SCFGR;
	uint32_t SPI_I2SPR;


}SPI_Regdef;

#define SPI1 ((SPI_Regdef*)SPI1_BASEADDR)
#define SPI2 ((SPI_Regdef*)SPI2_BASEADDR)
#define SPI3 ((SPI_Regdef*)SPI3_BASEADDR)
 /* USE EXAMPLE

  GPIOx_def_struct *pGPIOA =  (GPIOx_def_struct*)GPIOA_BASEADDR;
  pGPIOA->CRL = 25;

    OR

 #define GPIOA (GPIOx_def_struct*) GPIOA_BASEADDR
 GPIOx_def_struct *pGPIOA = GPIOA;
  GPIOA->CRL = 25;
 */

/* GPIO PERIPHERAL ADDRESSES TYPECASTING TO GPIOx_def_struct*/

#define GPIOA   ((GPIO_def_struct*)GPIOA_BASEADDR)
#define GPIOB   ((GPIO_def_struct*)GPIOB_BASEADDR)
#define GPIOC   ((GPIO_def_struct*)GPIOC_BASEADDR)
#define GPIOD   ((GPIO_def_struct*)GPIOD_BASEADDR)
#define GPIOE   ((GPIO_def_struct*)GPIOE_BASEADDR)
#define GPIOF   ((GPIO_def_struct*)GPIOF_BASEADDR)
#define GPIOG   ((GPIO_def_struct*)GPIOG_BASEADDR)

#define RCC     ((RCC_Regdef*)RCC_BASEADDR)


/****************************************************************CLOCK ENABLE MACROS***********************************************************/

#define GPIOA_CLK_EN()          (RCC->RCC_APB2ENR |= ( 2 << 1 ))
#define GPIOB_CLK_EN()          (RCC->RCC_APB2ENR |= ( 3 << 1 ))
#define GPIOC_CLK_EN()          (RCC->RCC_APB2ENR |= ( 4 << 1 ))
#define GPIOD_CLK_EN()          (RCC->RCC_APB2ENR |= ( 5 << 1 ))
#define GPIOE_CLK_EN()          (RCC->RCC_APB2ENR |= ( 6 << 1 ))

#define AFIO_CLK_EN()           (RCC->RCC_APB2ENR |= ( 0 << 1 ))

#define ADC1_CLK_EN()           (RCC->RCC_APB2ENR |= ( 9 << 1 ))
#define ADC2_CLK_EN()           (RCC->RCC_APB2ENR |= ( 10 << 1 ))

#define TIM1_CLK_EN()           (RCC->RCC_APB2ENR =| ( 11 << 1 ))
#define TIM2_CLK_EN()           (RCC->RCC_APB1ENR =| ( 0 << 1 ))
#define TIM3_CLK_EN()           (RCC->RCC_APB1ENR =| ( 1 << 1 ))
#define TIM4_CLK_EN()           (RCC->RCC_APB1ENR =| ( 2 << 1 ))
#define TIM5_CLK_EN()           (RCC->RCC_APB1ENR =| ( 3 << 1 ))
#define TIM6_CLK_EN()           (RCC->RCC_APB1ENR =| ( 4 << 1 ))
#define TIM7_CLK_EN()           (RCC->RCC_APB1ENR =| ( 5 << 1 ))

#define WWDG_CLK_EN()           (RCC->RCC_APB1ENR =| ( 11 << 1 ))

#define SPI1_CLK_EN()           (RCC->RCC_APB2ENR |= ( 12 << 1 ))
#define SPI2_CLK_EN()           (RCC->RCC_APB1ENR |= ( 14 << 1 ))
#define SPI3_CLK_EN()           (RCC->RCC_APB1ENR |= ( 15 << 1 ))

#define USART1_CLK_EN()         (RCC->RCC_APB2ENR =| ( 14 << 1 ))
#define USART2_CLK_EN()         (RCC->RCC_APB1ENR =| ( 17 << 1 ))
#define USART3_CLK_EN()         (RCC->RCC_APB1ENR =| ( 18 << 1 ))
#define UART4_CLK_EN()          (RCC->RCC_APB1ENR =| ( 19 << 1 ))
#define UART5_CLK_EN()          (RCC->RCC_APB2ENR =| ( 20 << 1 ))

#define I2C1_CLK_EN()           (RCC->RCC_APB1ENR =| ( 21 << 1 ))
#define I2C2_CLK_EN()           (RCC->RCC_APB1ENR =| ( 22 << 1 ))

#define CAN1_CLK_EN()           (RCC->RCC_APB1ENR =| ( 25 << 1 ))
#define CAN2_CLK_EN()           (RCC->RCC_APB1ENR =| ( 26 << 1 ))

#define BKP_CLK_EN()            (RCC->RCC_APB1ENR =| ( 27 << 1 ))

#define PWR_CLK_EN()            (RCC->RCC_APB1ENR =| ( 28 << 1 ))

#define DAC_CLK_EN()            (RCC->RCC_APB1ENR =| ( 29 << 1 ))


/****************************************************************CLOCK DISABLE MACROS***********************************************************/

#define GPIOA_CLK_DI()          (RCC->RCC_APB2ENR &= ~( 2 << 1 ))
#define GPIOB_CLK_DI()          (RCC->RCC_APB2ENR &= ~( 3 << 1 ))
#define GPIOC_CLK_DI()          (RCC->RCC_APB2ENR &= ~( 4 << 1 ))
#define GPIOD_CLK_DI()          (RCC->RCC_APB2ENR &= ~( 5 << 1 ))
#define GPIOE_CLK_DI()          (RCC->RCC_APB2ENR &= ~( 6 << 1 ))

#define AFIO_CLK_DI()           (RCC->RCC_APB2ENR &= ~( 0 << 1 ))

#define ADC1_CLK_DI()           (RCC->RCC_APB2ENR &= ~( 9 << 1 ))
#define ADC2_CLK_DI()           (RCC->RCC_APB2ENR &= ~( 10 << 1 ))

#define TIM1_CLK_DI()           (RCC->RCC_APB2ENR &= ~( 11 << 1 ))
#define TIM2_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 0 << 1 ))
#define TIM3_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 1 << 1 ))
#define TIM4_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 2 << 1 ))
#define TIM5_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 3 << 1 ))
#define TIM6_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 4 << 1 ))
#define TIM7_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 5 << 1 ))

#define WWDG_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 11 << 1 ))

#define SPI1_CLK_DI()           (RCC->RCC_APB2ENR &= ~( 12 << 1 ))
#define SPI2_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 14 << 1 ))
#define SPI3_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 15 << 1 ))

#define USART1_CLK_DI()         (RCC->RCC_APB2ENR &= ~( 14 << 1 ))
#define USART2_CLK_DI()         (RCC->RCC_APB1ENR &= ~( 17 << 1 ))
#define USART3_CLK_DI()         (RCC->RCC_APB1ENR &= ~( 18 << 1 ))
#define UART4_CLK_DI()          (RCC->RCC_APB1ENR &= ~( 19 << 1 ))
#define UART5_CLK_DI()          (RCC->RCC_APB2ENR &= ~( 20 << 1 ))

#define I2C1_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 21 << 1 ))
#define I2C2_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 22 << 1 ))

#define CAN1_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 25 << 1 ))
#define CAN2_CLK_DI()           (RCC->RCC_APB1ENR &= ~( 26 << 1 ))

#define BKP_CLK_DI()            (RCC->RCC_APB1ENR &= ~( 27 << 1 ))

#define PWR_CLK_DI()            (RCC->RCC_APB1ENR &= ~( 28 << 1 ))

#define DAC_CLK_DI()            (RCC->RCC_APB1ENR &= ~( 29 << 1 ))

/*
 * GPIO port reset macros
 */

#define GPIOA_REG_RESET()   do{ RCC->RCC_APB2RSTR |= (1 << 2); RCC->RCC_APB2RSTR &= ~(1 << 2);} while(0)
#define GPIOB_REG_RESET()   do{ RCC->RCC_APB2RSTR |= (1 << 3); RCC->RCC_APB2RSTR &= ~(1 << 3);} while(0)
#define GPIOC_REG_RESET()   do{ RCC->RCC_APB2RSTR |= (1 << 4); RCC->RCC_APB2RSTR &= ~(1 << 4);} while(0)
#define GPIOD_REG_RESET()   do{ RCC->RCC_APB2RSTR |= (1 << 5); RCC->RCC_APB2RSTR &= ~(1 << 5);} while(0)
#define GPIOE_REG_RESET()   do{ RCC->RCC_APB2RSTR |= (1 << 6); RCC->RCC_APB2RSTR &= ~(1 << 6);} while(0)

/*
 * SPI peripheral reset macros
 */

#define SPI1_REG_RESET()   do{ RCC->RCC_APB2RSTR |= (1 << 12); RCC->RCC_APB2RSTR &= ~(1 << 12);} while(0)
#define SPI2_REG_RESET()   do{ RCC->RCC_APB1RSTR |= (1 << 14); RCC->RCC_APB1RSTR &= ~(1 << 14);} while(0)
#define SPI3_REG_RESET()   do{ RCC->RCC_APB1RSTR |= (1 << 15); RCC->RCC_APB1RSTR &= ~(1 << 15);} while(0)
#endif /* INC_STM32F103XX_H_ */
