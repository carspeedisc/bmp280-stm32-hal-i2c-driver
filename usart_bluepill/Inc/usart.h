/*
 * usart.h
 *
 *  Created on: Jun 4, 2025
 *      Author: Rajiv
 */

#ifndef USART_H_
#define USART_H_



#define USART2_BASEADDR         0x40004400


/* USART REGISTERS OFFSET */

#define USART_SR_OFFSET         0x00
#define USART_DR_OFFSET         0x04
#define USART_BRR_OFFSET        0x08
#define USART_CR1_OFFSET        0x0C
#define USART_CR2_OFFSET        0x10
#define USART_CR3_OFFSET        0x14
#define USART_GTPR_OFFSET       0x18


#define USART2_SR               (*(volatile uint32_t*)(USART2_BASEADDR + USART_SR_OFFSET))
#define USART2_DR               (*(volatile uint32_t*)(USART2_BASEADDR + USART_DR_OFFSET))
#define USART2_BRR              (*(volatile uint32_t*)(USART2_BASEADDR + USART_BRR_OFFSET))
#define USART2_CR1              (*(volatile uint32_t*)(USART2_BASEADDR + USART_CR1_OFFSET))
#define USART2_CR2              (*(volatile uint32_t*)(USART2_BASEADDR + USART_CR2_OFFSET))
#define USART2_CR3              (*(volatile uint32_t*)(USART2_BASEADDR + USART_CR3_OFFSET))
#define USART2_GTPR             (*(volatile uint32_t*)(USART2_BASEADDR + USART_GTPR_OFFSET))


/*USART_SR REG BITMASKS*/

#define TXE                     (1<<7)






/*USART_CR1 REG BITMASKS*/

#define TE                      (1 << 3)   // Transmitter enable
#define RE                      (1 << 2)   // Receiver enable
#define UE                      (1 << 13)  // USART enable

#define WORD_LENGTH_8BITS       (0 << 12)  // M = 0 → 8 data bits
#define WORD_LENGTH_9BITS       (1 << 12)  // M = 1 → 9 data bits

#define PARITY_CONTROL_EN       (1 << 10)  // PCE: Parity control enable
#define EVEN_PARITY             (0 << 9)   // PS = 0 → Even parity
#define ODD_PARITY              (1 << 9)   // PS = 1 → Odd parity

#define SEND_BREAK              (1 << 0)   // SBK: Send break (bit 0)


/*USART_CR2 REG BITMASKS*/

#define STOP_BITS_1             (0 << 12)  // STOP[1:0] = 00
#define STOP_BITS_HALF          (1 << 12)  // STOP[1:0] = 01 (0.5 stop bit)
#define STOP_BITS_2             (2 << 12)  // STOP[1:0] = 10
#define STOP_BITS_1_5           (3 << 12)  // STOP[1:0] = 11

#define CLOCK_EN                (1 << 11)  // CLKEN: Clock enable


/*USART_CR3 REG BITMASKS*/

#define CTS_EN                  (1 << 9)   // CTSE: CTS enable
#define RTS_EN                  (1 << 8)   // RTSE: RTS enable
#define HALF_DUPLEX_COMM        (1 << 3)   // HDSEL: Half-duplex selection


void usart_init(void);
void uart_send_char(char c);
void delay(volatile unsigned int t);
















#endif /* USART_H_ */
