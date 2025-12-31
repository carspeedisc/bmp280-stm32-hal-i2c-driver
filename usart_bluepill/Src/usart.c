/*
 * usart.c
 *
 *  Created on: Jun 7, 2025
 *      Author: Rajiv
 */

#include"usart.h"
#include"gpio.h"
#include"stm32f103.h"
void usart_init(void){


	GPIOA_CLK_EN();
	USART2_CLK_EN();

	GPIOx_CRL &= ~(0xFFFF);
	GPIOx_CRL |= (0xBB<<8);


	//USART2_BRR = (234 << 4) | 6;
	USART2_BRR = (52 << 4) | 1;   // Correct for 8 MHz clock
   // Correct for 8 MHz → 9600 baud

	USART2_CR1 |= UE | TE | RE;
}

void uart_send_char(char c) {

    while (!(USART2_SR & TXE));
    USART2_DR |= c;
}

void delay(volatile unsigned int t) {
    while (t--) __asm__("nop");
}
