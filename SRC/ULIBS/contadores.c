#include "contadores.h"
#include <avr/interrupt.h>

#define PF2_INTERRUPT               ( PORTF.INTFLAGS & PIN2_bm )
#define PF2_CLEAR_INTERRUPT_FLAG    ( PORTF.INTFLAGS &= PIN2_bm )

#define PE6_INTERRUPT               ( PORTE.INTFLAGS & PIN6_bm )
#define PE6_CLEAR_INTERRUPT_FLAG    ( PORTE.INTFLAGS &= PIN6_bm )

#define CNT0_PORT	PORTF
#define CNT0_PIN    2   
#define CNT0_PIN_bm	PIN2_bm
#define CNT0_PIN_bp	PIN2_bp
#define CNT0_PIN_CONFIG()           PORTF.PIN2CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

void pv_count_pulse(void);

TaskHandle_t *xHandlePtr;

// Cada minuto guardo los pulsos en un slot.
#define PBUFFER_LENGTH 60
uint8_t idx;
uint16_t pbuffer[PBUFFER_LENGTH];
uint16_t tmp_cnt;


//------------------------------------------------------------------------------
void counter_init_outofrtos( TaskHandle_t *xHandle )
{
    /*
     * El pin del contador es INPUT, PULLOPEN e interrumpe en flanco de bajada.
     * 
     * Utilizo el handler de tkCtl para mostrar el debug.
     * 
     */
    
    xHandlePtr = xHandle;
    // Los CNTx son inputs
    CNT0_PORT.DIR &= ~CNT0_PIN_bm;
    // Habilito a interrumpir, pullup, flanco de bajada.
    cli();
    CNT0_PIN_CONFIG();
    //PORTF.PIN4CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
    sei();
    
    contador.fsm_ticks_count = 0;
    for (idx=0; idx<PBUFFER_LENGTH; idx++) {
        pbuffer[idx] = 0;
    }
    idx = 0;
    tmp_cnt = 0;
}
// ----------------------------------------------------------------------------- 
void counter_config_timerpoll( char *s_timerpoll)
{
    counter_conf.timerpoll = atoi(s_timerpoll);
}
// ----------------------------------------------------------------------------- 
void counter_config_magpp( char *s_magpp)
{
    counter_conf.magpp = atof(s_magpp);
}
// ----------------------------------------------------------------------------- 
uint8_t counter_read_pin(void)
{
    return ( ( CNT0_PORT.IN & CNT0_PIN_bm ) >> CNT0_PIN) ;
}
// -----------------------------------------------------------------------------
void counter_read(counter_value_t *cnt)
{
    memcpy( cnt, &contador, sizeof(counter_value_t));
}
// -----------------------------------------------------------------------------
void counter_clear(void)
{
   /*
    * Una vez por periodo ( timerpoll ) borro los contadores.
    * Si en el periodo NO llegaron pulsos, aqui debo entonces en los
    * caudales agregar un 0.0 al ring buffer para que luego de un tiempo
    * converja a 0.
    * 
    */
    contador.caudal = 0.0;
    contador.pulsos = 0;
}
//------------------------------------------------------------------------------
void counter_summarize(void)
{
    /*
     * Esta función la invoca una base de tiempos 1 vez por minuto
     * Guarda el valor de los pulsos en dicho minuto, y lo almacena en el
     * buffer para tener los pulsos x hora.
     */
    
uint8_t i;
uint32_t pXh = 0;

    // Guardo en el buffer de pulsosxhora los pulsos del ultimo minuto
    pbuffer[idx] = tmp_cnt;
    // Actualizo variables
    contador.pulsosXmin = tmp_cnt;
    tmp_cnt = 0;
    
    // Avanzo el puntero en modo circular
    if ( ++idx == PBUFFER_LENGTH ) {
        idx = 0;
    }
    
    // Calculo la cantidad de pulsos en la ultima hora
    for (i=0; i < PBUFFER_LENGTH; i++) {
        pXh += pbuffer[i];
    }
    contador.pulsosXhora = pXh;
    
    xprintf_P(PSTR("SUMMARY: ppm=%lu, pph=%lu\r\n"), contador.pulsosXmin, contador.pulsosXhora);
    
}
//------------------------------------------------------------------------------
void pv_count_pulse(void)
{
  
BaseType_t xHigherPriorityTaskWoken = pdTRUE;

    contador.now_ticks = xTaskGetTickCountFromISR();
    contador.T_ticks = contador.now_ticks - contador.start_pulse_ticks;
    // Guardo el inicio del pulso para medir el caudal del proximo pulso
    contador.start_pulse_ticks = contador.now_ticks;
    
    // No cuento pulsos menores de 5 ticks de ancho (5ms)
    if (contador.T_ticks < 5 ) {
        return;
    }
    
    tmp_cnt++;
    contador.pulsos++;
    contador.T_secs =  (float)(1.0 * contador.T_ticks);    // Duracion en ticks
    contador.T_secs /= configTICK_RATE_HZ;                 // Duracion en secs.

    if ( contador.T_secs > 0 ) {
        contador.caudal = counter_conf.magpp / contador.T_secs;      // En mt3/s 
        contador.caudal *= 3600;                                     // En mt3/h
    } else {
        contador.caudal = 0.0;
    } 
            
    xTaskNotifyFromISR( *xHandlePtr,
                       0x01,
                       eSetBits,
                       &xHigherPriorityTaskWoken );
            
}
//------------------------------------------------------------------------------
/*
 * Note: If the flag is not cleared, the interrupt will keep triggering, so 
 * it is essential that the flag is always cleared before exiting the ISR. 
 * Any algorithm that relies on not clearing the interrupt flag is highly 
 * discouraged because this effectively overloads the ISR responsibility. 
 * The resulting responsibilities of the ISR will be to handle the interrupt 
 * and to keep firing until the flag is cleared. 
 * This violates the Single Responsibility principle in software design 
 * and extreme care must be taken to avoid bugs.
 */
ISR(PORTF_PORT_vect)
{
    // Borro las flags.
    if (PF2_INTERRUPT) {
        //led_toggle();
        pv_count_pulse();
                    
        // Se borra la flag de interrupcion para habilitarla de nuevo
        // Si no la borro antes de salir, se vuelve a generar la int.
        PF2_CLEAR_INTERRUPT_FLAG;
    }

}
     