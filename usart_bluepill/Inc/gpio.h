/*
 * gpio.h
 *
 *  Created on: Jun 7, 2025
 *      Author: Rajiv
 */

#ifndef GPIO_H_
#define GPIO_H_

#include<stdint.h>


#define GPIOA_BASEADDR                 0x40010800

/*GPIO REGISTERS OFFSET*/

#define GPIOx_CRL_OFFSET               0x00
#define GPIOx_CRH_OFFSET               0x04
#define GPIOx_IDR_OFFSET               0x08
#define GPIOx_ODR_OFFSET               0x0C
#define GPIOx_BSRR_OFFSET              0x10
#define GPIOx_BRR_OFFSET               0x14
#define GPIOx_LCKR_OFFSET              0x18


/*GPIOA REGISTERS DEFINITION*/

#define GPIOx_CRL                      (*(volatile uint32_t*)(GPIOA_BASEADDR + GPIOx_CRL_OFFSET))
#define GPIOx_CRH                      (*(volatile uint32_t*)(GPIOA_BASEADDR + GPIOx_CRH_OFFSET))
#define GPIOx_IDR                      (*(volatile uint32_t*)(GPIOA_BASEADDR + GPIOx_IDR_OFFSET))
#define GPIOx_ODR                      (*(volatile uint32_t*)(GPIOA_BASEADDR + GPIOx_ODR_OFFSET))
#define GPIOx_BSRR                     (*(volatile uint32_t*)(GPIOA_BASEADDR + GPIOx_BSRR_OFFSET))
#define GPIOx_BRR                      (*(volatile uint32_t*)(GPIOA_BASEADDR + GPIOx_BRR_OFFSET))
#define GPIOx_LCKR                     (*(volatile uint32_t*)(GPIOA_BASEADDR + GPIOx_LCKR_OFFSET))


#define ALTERNATE_FUN_PP                0xB
#define ALTERNATE_FUN_OD                0xF



#define OUTPUT_PP                       0x1
#define OUTPUT_OD                       0x6


#define INPUT_ANALOG                    0x0
#define INPUT_FLOATING                  0x4
#define INPUT_PUPD                      0x8















#endif /* GPIO_H_ */
