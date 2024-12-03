#include "PULSE2MBUS.h"


//------------------------------------------------------------------------------
void tkSys(void * pvParameters)
{

	/*
     *  Genera una espera exacta de 1 minuto y luego actualiza los contadores.
     *  para tener el valor xminuto y xhora
     */

( void ) pvParameters;

    while ( ! starting_flag )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

    // Marco la tarea activa
    SYSTEM_ENTER_CRITICAL();
    tk_running[TK_SYS] = true;
    SYSTEM_EXIT_CRITICAL();

	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
    xprintf_P(PSTR("Starting tkSys..\r\n"));
    
	for( ;; )
	{
        u_kick_wdt(TK_SYS);      
        counter_summarize();
        vTaskDelay( ( TickType_t)( 60000 / portTICK_PERIOD_MS ) );
            
	}
}
//------------------------------------------------------------------------------

