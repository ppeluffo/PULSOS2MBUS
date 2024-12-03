    /* 
 * File:   pines.h
 * Author: pablo
 *
 * Created on 11 de febrero de 2022, 06:02 PM
 */

#ifndef PINES_H
#define	PINES_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <avr/io.h>
#include "stdbool.h"
    
//--------------------------------------------------------------------------

// TERM_SENSE (INPUT)
#define TERM_SENSE_PORT         PORTG
#define TERM_SENSE_PIN              7
#define TERM_SENSE_PIN_bm       PIN7_bm
#define TERM_SENSE_PIN_bp       PIN7_bp
#define CONFIG_TERM_SENSE()     TERM_SENSE_PORT.DIR &= ~TERM_SENSE_PIN_bm;

#define READ_TERM_SENSE()       ( ( TERM_SENSE_PORT.IN & TERM_SENSE_PIN_bm ) >> TERM_SENSE_PIN)
    
    
// RS485_RTS
#define RTS_RS485_PORT         PORTE
#define RTS_RS485              2
#define RTS_RS485_PIN_bm       PIN2_bm
#define RTS_RS485_PIN_bp       PIN2_bp
#define SET_RTS_RS485()        ( RTS_RS485_PORT.OUT |= RTS_RS485_PIN_bm )
#define CLEAR_RTS_RS485()      ( RTS_RS485_PORT.OUT &= ~RTS_RS485_PIN_bm )
#define CONFIG_RTS_485()       RTS_RS485_PORT.DIR |= RTS_RS485_PIN_bm;
    
// AWAKE (INPUT)
#define AWAKE_SENSE_PORT         PORTC
#define AWAKE_SENSE_PIN              1
#define AWAKE_SENSE_PIN_bm       PIN1_bm
#define AWAKE_SENSE_PIN_bp       PIN1_bp
#define CONFIG_AWAKE_SENSE()     AWAKE_SENSE_PORT.DIR &= ~AWAKE_SENSE_PIN_bm;

#define READ_AWAKE_SENSE()       ( ( AWAKE_SENSE_PORT.IN & AWAKE_SENSE_PIN_bm ) >> AWAKE_SENSE_PIN)  

#ifdef	__cplusplus
}
#endif

#endif	/* PINES_H */

