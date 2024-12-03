/* 
 * File:   contadores.h
 * Author: pablo
 *
 * Created on July 19, 2023, 5:04 PM
 */

#ifndef CONTADORES_H
#define	CONTADORES_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
    
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "math.h"
    
#include "xprintf.h"    
#include "pines.h"
#include "utils.h"
    
// Configuracion de canales de contadores
typedef struct {
	float magpp;
    uint16_t timerpoll;  
} counter_conf_t;

counter_conf_t counter_conf;

typedef struct {
    uint32_t pulsosXmin;
    uint32_t pulsosXhora;
    uint16_t pulsos;
    float caudal;
    uint8_t fsm_ticks_count;
    uint32_t start_pulse_ticks;
    uint32_t now_ticks;
    uint32_t T_ticks;
    float T_secs;
    
} counter_value_t;

counter_value_t contador;

StaticTimer_t counter_xTimerBuffer;
TimerHandle_t counter_xTimer;

void counter_init_outofrtos(TaskHandle_t *xHandle);
void counter_config_timerpoll( char *s_timerpoll);
void counter_config_magpp( char *s_magpp);
void counter_clear(void);
uint8_t counter_read_pin(void);
void counter_read(counter_value_t *cnt);
void counter_summarize(void);


#ifdef	__cplusplus
}
#endif

#endif	/* CONTADORES_H */
