/*
 * drv_uart_spx.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 * 
 * PLACA BASE: sensor_cloro.
 * 
 * El driver de las uart permite crear las uarts y 2 estructuras tipo 
 * ringbuffer (chars) para c/u.
 * Estos son las interfaces a la capa de FRTOS-IO.
 * Para transmitir se escribe en el ringBuffer de TX y para leer lo recibido
 * se lee del ringBuffer de RX.
 * La transmision / recepcion se hace por interrupcion. Estas ISR son 
 * provistas por el driver
 * Cada placa tiene diferente asignacion de puertos por lo tanto hay
 * que modificar el driver a c/placa.
 * 
 * 
 */

#include "drv_uart_avrDX.h"

//------------------------------------------------------------------------------
// USART0: TERM
//------------------------------------------------------------------------------
void drv_uart0_init(uint32_t baudrate )
{
    
    PORTA.DIR &= ~PIN1_bm;
    PORTA.DIR |= PIN0_bm;
    USART0.BAUD = (uint16_t)USART_SET_BAUD_RATE(baudrate);     
    USART0.CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;
    
    // Habilito el TX y el RX
    USART0.CTRLB |= USART_TXEN_bm;
    USART0.CTRLB |= USART_RXEN_bm;
    
    // Habilito las interrupciones por RX
    USART0.CTRLA |= USART_RXCIE_bm;
    
    // Las transmisiones son por poleo no INT.
    
    // RingBuffers
    rBchar_CreateStatic ( &TXRB_uart0, &uart0_txBuffer[0], UART0_TXSIZE  );
    rBchar_CreateStatic ( &RXRB_uart0, &uart0_rxBuffer[0], UART0_RXSIZE  );
}
//------------------------------------------------------------------------------
ISR(USART0_RXC_vect)
{
    // Driver ISR: Cuando se genera la interrupcion por RXIE, lee el dato
    // y lo pone en la cola (ringBuffer.)
char cChar = ' ';

	cChar = USART0.RXDATAL;
 	rBchar_PokeFromISR( &RXRB_uart0, cChar );
}
//------------------------------------------------------------------------------
// USART4: RS485
//------------------------------------------------------------------------------
void drv_uart4_init(uint32_t baudrate )
{
    
    PORTE.DIR &= ~PIN1_bm;
    PORTE.DIR |= PIN0_bm;
    USART4.BAUD = (uint16_t)USART_SET_BAUD_RATE(baudrate);     
    USART4.CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;
    
    // Habilito el TX y el RX
    USART4.CTRLB |= USART_TXEN_bm;
    USART4.CTRLB |= USART_RXEN_bm;
    
    // Habilito las interrupciones por RX
    USART4.CTRLA |= USART_RXCIE_bm;
    
    // Las transmisiones son por poleo no INT.
    
    // RingBuffers
    rBchar_CreateStatic ( &TXRB_uart4, &uart4_txBuffer[0], UART4_TXSIZE  );
    rBchar_CreateStatic ( &RXRB_uart4, &uart4_rxBuffer[0], UART4_RXSIZE  );
}
//------------------------------------------------------------------------------
ISR(USART4_RXC_vect)
{
    // Driver ISR: Cuando se genera la interrupcion por RXIE, lee el dato
    // y lo pone en la cola (ringBuffer.)
char cChar = ' ';

	cChar = USART4.RXDATAL;
 	rBchar_PokeFromISR( &RXRB_uart4, cChar );
}
//------------------------------------------------------------------------------
