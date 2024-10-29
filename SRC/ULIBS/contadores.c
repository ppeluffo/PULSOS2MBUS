#include "contadores.h"
#include <avr/interrupt.h>

#define PF4_INTERRUPT               ( PORTF.INTFLAGS & PIN4_bm )
#define PF4_CLEAR_INTERRUPT_FLAG    ( PORTF.INTFLAGS &= PIN4_bm )

#define PE6_INTERRUPT               ( PORTE.INTFLAGS & PIN6_bm )
#define PE6_CLEAR_INTERRUPT_FLAG    ( PORTE.INTFLAGS &= PIN6_bm )

#define CNT0_PORT	PORTF
#define CNT0_PIN    4   
#define CNT0_PIN_bm	PIN4_bm
#define CNT0_PIN_bp	PIN4_bp
#define CNT0_PIN_CONFIG()           PORTF.PIN4CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

void pv_count_pulse(void);

TaskHandle_t *l_xHandle_tkCtl;

//------------------------------------------------------------------------------
void counter_init_outofrtos( TaskHandle_t *xHandle )
{
    /*
     * El pin del contador es INPUT, PULLOPEN e interrumpe en flanco de bajada.
     * 
     * Utilizo el handler de tkCtl para mostrar el debug.
     * 
     */
    
    l_xHandle_tkCtl = xHandle;
    // Los CNTx son inputs
    CNT0_PORT.DIR &= ~CNT0_PIN_bm;
    // Habilito a interrumpir, pullup, flanco de bajada.
    cli();
    CNT0_PIN_CONFIG();
    //PORTF.PIN4CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
    sei();
    
    contador.fsm_ticks_count = 0;
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
counter_value_t counter_read(void)
{
    return(contador);
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
void pv_count_pulse(void)
{
  
BaseType_t xHigherPriorityTaskWoken = pdTRUE;

    contador.now_ticks = xTaskGetTickCountFromISR();
    contador.T_ticks = contador.now_ticks - contador.start_pulse_ticks;
    // Guardo el inicio del pulso para medir el caudal del proximo pulso
    contador.start_pulse_ticks = contador.now_ticks;
    
    // No cuento pulsos menores de 100 ticks de ancho
    if (contador.T_ticks < 10) {
        return;
    }
    
    contador.pulsos++;
    contador.T_secs =  (float)(1.0 * contador.T_ticks);    // Duracion en ticks
    contador.T_secs /= configTICK_RATE_HZ;                 // Duracion en secs.

    if ( contador.T_secs > 0 ) {
        contador.caudal = counter_conf.magpp / contador.T_secs;      // En mt3/s 
        contador.caudal *= 3600;                                     // En mt3/h
    } else {
        contador.caudal = 0.0;
    } 
            
    xTaskNotifyFromISR( *l_xHandle_tkCtl,
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
    if (PF4_INTERRUPT) {
        //led_toggle();
        pv_count_pulse();
                    
        // Se borra la flag de interrupcion para habilitarla de nuevo
        // Si no la borro antes de salir, se vuelve a generar la int.
        PF4_CLEAR_INTERRUPT_FLAG;
    }

}
     