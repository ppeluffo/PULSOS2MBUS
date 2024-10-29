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
#define RTS_RS485_PORT         PORTD
#define RTS_RS485              7
#define RTS_RS485_PIN_bm       PIN7_bm
#define RTS_RS485_PIN_bp       PIN7_bp
#define SET_RTS_RS485()        ( RTS_RS485_PORT.OUT |= RTS_RS485_PIN_bm )
#define CLEAR_RTS_RS485()      ( RTS_RS485_PORT.OUT &= ~RTS_RS485_PIN_bm )
#define CONFIG_RTS_485()       RTS_RS485_PORT.DIR |= RTS_RS485_PIN_bm;
    
// EN_PWR_CPRES (output)
#define EN_PWR_CPRES_PORT         PORTD
#define EN_PWR_CPRES              6
#define EN_PWR_CPRES_PIN_bm       PIN6_bm
#define EN_PWR_CPRES_PIN_bp       PIN6_bp
#define SET_EN_PWR_CPRES()        ( EN_PWR_CPRES_PORT.OUT |= EN_PWR_CPRES_PIN_bm )
#define CLEAR_EN_PWR_CPRES()      ( EN_PWR_CPRES_PORT.OUT &= ~EN_PWR_CPRES_PIN_bm )
#define CONFIG_EN_PWR_CPRES()     EN_PWR_CPRES_PORT.DIR |= EN_PWR_CPRES_PIN_bm;
    
// EN_PWR_SENSEXT (output)
#define EN_PWR_SENSEXT_PORT         PORTF
#define EN_PWR_SENSEXT              0
#define EN_PWR_SENSEXT_PIN_bm       PIN0_bm
#define EN_PWR_SENSEXT_PIN_bp       PIN0_bp
#define SET_EN_PWR_SENSEXT()        ( EN_PWR_SENSEXT_PORT.OUT |= EN_PWR_SENSEXT_PIN_bm )
#define CLEAR_EN_PWR_SENSEXT()      ( EN_PWR_SENSEXT_PORT.OUT &= ~EN_PWR_SENSEXT_PIN_bm )
#define CONFIG_EN_PWR_SENSEXT()     EN_PWR_SENSEXT_PORT.DIR |= EN_PWR_SENSEXT_PIN_bm;

// EN_PWR_QMBUS (output)
#define EN_PWR_QMBUS_PORT         PORTF
#define EN_PWR_QMBUS              1
#define EN_PWR_QMBUS_PIN_bm       PIN1_bm
#define EN_PWR_QMBUS_PIN_bp       PIN1_bp
#define SET_EN_PWR_QMBUS()        ( EN_PWR_QMBUS_PORT.OUT |= EN_PWR_QMBUS_PIN_bm )
#define CLEAR_EN_PWR_QMBUS()      ( EN_PWR_QMBUS_PORT.OUT &= ~EN_PWR_QMBUS_PIN_bm )
#define CONFIG_EN_PWR_QMBUS()     EN_PWR_QMBUS_PORT.DIR |= EN_PWR_QMBUS_PIN_bm;
    
// PWR_SENSORS (sensors de 4-20, output)
#define PWR_SENSORS_PORT         PORTC
#define PWR_SENSORS              1
#define PWR_SENSORS_PIN_bm       PIN1_bm
#define PWR_SENSORS_PIN_bp       PIN1_bp
#define SET_PWR_SENSORS()        ( PWR_SENSORS_PORT.OUT |= PWR_SENSORS_PIN_bm )
#define CLEAR_PWR_SENSORS()      ( PWR_SENSORS_PORT.OUT &= ~PWR_SENSORS_PIN_bm )
#define CONFIG_PWR_SENSORS()     PWR_SENSORS_PORT.DIR |= PWR_SENSORS_PIN_bm;
    
// EN_SENS3V3 (output)
#define EN_SENS3V3_PORT         PORTD
#define EN_SENS3V3              5
#define EN_SENS3V3_PIN_bm       PIN5_bm
#define EN_SENS3V3_PIN_bp       PIN5_bp
#define SET_EN_SENS3V3()        ( EN_SENS3V3_PORT.OUT |= EN_SENS3V3_PIN_bm )
#define CLEAR_EN_SENS3V3()      ( EN_SENS3V3_PORT.OUT &= ~EN_SENS3V3_PIN_bm )
#define CONFIG_EN_SENS3V3()     EN_PWR_CPRES_PORT.DIR |= EN_SENS3V3_PIN_bm;
    
// EN_SENS12V (output)
#define EN_SENS12V_PORT         PORTD
#define EN_SENS12V              4
#define EN_SENS12V_PIN_bm       PIN4_bm
#define EN_SENS12V_PIN_bp       PIN4_bp
#define SET_EN_SENS12V()        ( EN_SENS12V_PORT.OUT |= EN_SENS12V_PIN_bm )
#define CLEAR_EN_SENS12V()      ( EN_SENS12V_PORT.OUT &= ~EN_SENS12V_PIN_bm )
#define CONFIG_EN_SENS12V()     EN_PWR_CPRES_PORT.DIR |= EN_SENS12V_PIN_bm;
    

#ifdef	__cplusplus
}
#endif

#endif	/* PINES_H */

