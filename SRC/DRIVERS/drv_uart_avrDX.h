/*
 * drv_uart_spx.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_DRIVERS_DRV_UART_SPX_H_
#define SRC_SPX_DRIVERS_DRV_UART_SPX_H_

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "FreeRTOS.h"

#include "ringBuffer.h"


#ifndef F_CPU
#define F_CPU 24000000
#endif

#define USART_SET_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5);

//-----------------------------------------------------------------------
#define UART0_TXSIZE	8	// trasmito por poleo. Si uso interrupcion lo subo a 128
uint8_t uart0_txBuffer[UART0_TXSIZE];
#define UART0_RXSIZE	128	// Este UART es el que atiende el modem.
uint8_t uart0_rxBuffer[UART0_RXSIZE];
rBchar_s TXRB_uart0, RXRB_uart0;
void drv_uart0_init(uint32_t baudrate );

#define UART4_TXSIZE	8	// trasmito por poleo. Si uso interrupcion lo subo a 128
uint8_t uart4_txBuffer[UART4_TXSIZE];
#define UART4_RXSIZE	128	// 
uint8_t uart4_rxBuffer[UART4_RXSIZE];
rBchar_s TXRB_uart4, RXRB_uart4;
void drv_uart4_init(uint32_t baudrate );

//-----------------------------------------------------------------------


#endif /* SRC_SPX_DRIVERS_DRV_UART_SPX_H_ */
