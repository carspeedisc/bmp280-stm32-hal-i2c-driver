/*
 * stm32f103.h
 *
 *  Created on: Jun 9, 2025
 *      Author: Rajiv
 */

#ifndef STM32F103_H_
#define STM32F103_H_

#define AHB_BASEADDR                         0x40018000

#define RCC_BASEADDR                         0x40021000


/* RCC REGISTERS */

#define RCC_CR                           (*(volatile uint32_t*)(0x40021000 + 0x00))
#define RCC_CFGR                         (*(volatile uint32_t*)(0x40021000 + 0x04))
#define RCC_CIR                          (*(volatile uint32_t*)(0x40021000 + 0x08))
#define RCC_APB2RSTR                     (*(volatile uint32_t*)(0x40021000 + 0x0C))
#define RCC_APB1RSTR                     (*(volatile uint32_t*)(0x40021000 + 0x10))
#define RCC_AHBENR                       (*(volatile uint32_t*)(0x40021000 + 0x14))
#define RCC_APB2ENR                      (*(volatile uint32_t*)(0x40021000 + 0x18))
#define RCC_APB1ENR                      (*(volatile uint32_t*)(0x40021000 + 0x1C))
#define RCC_BDCR                         (*(volatile uint32_t*)(0x40021000 + 0x20))
#define RCC_CSR                          (*(volatile uint32_t*)(0x40021000 + 0x24))
#define RCC_AHBRSTR                      (*(volatile uint32_t*)(0x40021000 + 0x28))
#define RCC_CFGR2                        (*(volatile uint32_t*)(0x40021000 + 0x2C))



#define GPIOA_CLK_EN()      do { RCC_APB2ENR |= (1 << 2); } while(0)
#define USART2_CLK_EN()     do { RCC_APB1ENR |= (1 << 17); } while(0)













#endif /* STM32F103_H_ */
