#include "PULSE2MBUS.h"
#include "frtos_cmd.h"
#include "modbus_slave.h"

//------------------------------------------------------------------------------
void tkRS485RX(void * pvParameters)
{
    /*
     * Esta tarea recibe los datos de la interfase RS485 y los pasa
     * a la FSM de modbus
     */


( void ) pvParameters;
uint8_t c = 0;
uint32_t ulNotificationValue;

    while ( ! starting_flag )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

    SYSTEM_ENTER_CRITICAL();
    tk_running[TK_RS485RX] = true;
    SYSTEM_EXIT_CRITICAL();
    
	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );

    
    /*
     * Este tarea recibe los datos del puerto A que es donde esta el bus modbus.
     * Debo inicializar el sistema modbus !!!
     */
    // La variable tickless se fija externamente para controlar cuando entro y salgo del 
    // modo.
    
    modbus_slave_fsm_init();
    
    xprintf_P(PSTR("Starting tkRS485..\r\n" ));
    
	// loop
	for( ;; )
	{
        
        u_kick_wdt(TK_RS485RX);
        
        while ( true ) {
            c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
            // el read se bloquea 50ms. lo que genera la espera.
            while ( xfgetc( fdRS485, (char *)&c ) == 1 ) {
               modbus_slave_fsm(c);
            }
        
            //vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );     
            
            if (rs485_awake) {
                vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
            } else {
                ulNotificationValue = ulTaskNotifyTake(pdTRUE, ( TickType_t)( 60000 / portTICK_PERIOD_MS ));
                if( ( ulNotificationValue & 0x01 ) != 0 ) {
                    xprintf_P(PSTR("AWAKE signal !!\r\n"));  
                }           
            }
            
        }
	}    
}
//------------------------------------------------------------------------------
