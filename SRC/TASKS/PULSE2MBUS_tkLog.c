#include "PULSE2MBUS.h"


//------------------------------------------------------------------------------
void tkLog(void * pvParameters)
{

	/*
     *  Genera una espera exacta de 1 minuto y luego actualiza los contadores.
     *  para tener el valor xminuto y xhora
     */

( void ) pvParameters;
uint32_t ulNotificationValue;

    while ( ! starting_flag )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

    // Marco la tarea activa
    SYSTEM_ENTER_CRITICAL();
    tk_running[TK_LOG] = true;
    SYSTEM_EXIT_CRITICAL();

	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
    xprintf_P(PSTR("Starting tkSys..\r\n"));
   
	for( ;; )
	{
        u_kick_wdt(TK_LOG);
        
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, ( TickType_t)( 60000 / portTICK_PERIOD_MS ));
        if( ( ulNotificationValue ) != 0 ) {
            
            xprintf_P(PSTR("COUNTER: PULSOS=%d, CAUDAL=%0.3f, PW(secs)=%0.3f\r\n"), contador.pulsos, contador.caudal, contador.T_secs );
            //xprintf_P(PSTR("COUNTER**: TNOW=%lu, PW(ticks)=%lu\r\n"),  contador.now_ticks,  contador.T_ticks );      
        }
                   
	}
}
//------------------------------------------------------------------------------


