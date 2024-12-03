/* 
 * File:   frtos20_utils.c
 * Author: pablo
 *
 * Created on 22 de diciembre de 2021, 07:34 AM
 */

#include "PULSE2MBUS.h"
#include "pines.h"

struct {
    uint8_t modbus_address;
    float magpp;
    uint16_t timerpoll;  
    uint8_t checksum;
} NVMBuffer;

//------------------------------------------------------------------------------
int8_t WDT_init(void);
int8_t CLKCTRL_init(void);
uint8_t checksum( uint8_t *s, uint16_t size );

//-----------------------------------------------------------------------------
void system_init()
{

    // Init OUT OF RTOS !!!
    
	CLKCTRL_init();
    //WDT_init();
    LED_init();
    XPRINTF_init();
    
    u_config_termsense();
       
    CONFIG_RTS_485();
    // RTS OFF: Habilita la recepcion del chip
	CLEAR_RTS_RS485();
    
    //CONFIG_AWAKE_SENSE();
    pin_awake_config();

}
//-----------------------------------------------------------------------------
int8_t WDT_init(void)
{
	/* 8K cycles (8.2s) */
	/* Off */
	ccp_write_io((void *)&(WDT.CTRLA), WDT_PERIOD_8KCLK_gc | WDT_WINDOW_OFF_gc );  
	return 0;
}
//-----------------------------------------------------------------------------
int8_t CLKCTRL_init(void)
{
	// Configuro el clock para 24Mhz
	
	ccp_write_io((void *)&(CLKCTRL.OSCHFCTRLA), CLKCTRL_FRQSEL_24M_gc         /* 24 */
                                                
	| 0 << CLKCTRL_AUTOTUNE_bp /* Auto-Tune enable: disabled */
	| 0 << CLKCTRL_RUNSTDBY_bp /* Run standby: disabled */);

	// ccp_write_io((void*)&(CLKCTRL.MCLKCTRLA),CLKCTRL_CLKSEL_OSCHF_gc /* Internal high-frequency oscillator */
	//		 | 0 << CLKCTRL_CLKOUT_bp /* System clock out: disabled */);

	// ccp_write_io((void*)&(CLKCTRL.MCLKLOCK),0 << CLKCTRL_LOCKEN_bp /* lock enable: disabled */);

	return 0;
}
//-----------------------------------------------------------------------------
void reset(void)
{
    xprintf_P(PSTR("ALERT !!!. Going to reset...\r\n"));
    vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
	/* Issue a Software Reset to initilize the CPU */
	ccp_write_io( (void *)&(RSTCTRL.SWRR), RSTCTRL_SWRST_bm ); 
                                           
}
//------------------------------------------------------------------------------
void u_kick_wdt( t_wdg_ids wdg_id)
{
    // Pone el wdg correspondiente en true
    tk_watchdog[wdg_id] = true;
    
}
//------------------------------------------------------------------------------
void u_config_default( void )
{

    systemConf.modbus_address = 100;
    counter_conf.magpp = 0.1;
    counter_conf.timerpoll = 60; 
}
//------------------------------------------------------------------------------
bool config_debug( char *tipo, char *valor)
{
    return(true);
}
//------------------------------------------------------------------------------
bool u_save_config_in_NVM(void)
{  
int8_t retVal;
uint8_t cks;

    memset( &NVMBuffer, '\0', sizeof(NVMBuffer));

    // Cargamos el buffer con las configuraciones
    NVMBuffer.modbus_address = systemConf.modbus_address;
    NVMBuffer.magpp = counter_conf.magpp;
    NVMBuffer.timerpoll = counter_conf.timerpoll;
    cks = checksum ( (uint8_t *)&NVMBuffer, ( sizeof(NVMBuffer) - 1));
    NVMBuffer.checksum = cks;

    retVal = NVMEE_write( 0x00, (char *)&NVMBuffer, sizeof(NVMBuffer) );

    xprintf_P(PSTR("SAVE NVM: memblock size = %d\r\n"), sizeof(NVMBuffer));    
    
    if (retVal == -1 )
        return(false);
    
    return(true);

}
//------------------------------------------------------------------------------
bool u_load_config_from_NVM(void)
{
uint8_t rd_cks, calc_cks;

    xprintf_P(PSTR("NVM: memblock size=%d\r\n"), sizeof(NVMBuffer));

    memset( &NVMBuffer, '\0', sizeof(NVMBuffer));
    
    NVMEE_read( 0x00, (char *)&NVMBuffer, sizeof(NVMBuffer) );
    rd_cks = NVMBuffer.checksum;
        
    calc_cks = checksum ( (uint8_t *)&NVMBuffer, ( sizeof(NVMBuffer) - 1));
    
    if ( calc_cks != rd_cks ) {
		xprintf_P( PSTR("ERROR: Checksum systemConf failed: calc[0x%0x], read[0x%0x]\r\n"), calc_cks, rd_cks );
        return(false);
	} 
    
    // Desarmo el buffer de memoria
    systemConf.modbus_address = NVMBuffer.modbus_address;
    counter_conf.magpp = NVMBuffer.magpp;
    counter_conf.timerpoll = NVMBuffer.timerpoll;
    
    mbus_local_address = systemConf.modbus_address;
    
    return(true);
}
//------------------------------------------------------------------------------
uint8_t checksum( uint8_t *s, uint16_t size )
{
	/*
	 * Recibe un puntero a una estructura y un tamaño.
	 * Recorre la estructura en forma lineal y calcula el checksum
	 */

uint8_t *p = NULL;
uint8_t cks = 0;
uint16_t i = 0;

	cks = 0;
	p = s;
	for ( i = 0; i < size ; i++) {
		 cks = (cks + (int)(p[i])) % 256;
	}

	return(cks);
}
//------------------------------------------------------------------------------
bool u_config_timerpoll ( char *s_timerpoll )
{
	return(true);
}
//------------------------------------------------------------------------------
void SYSTEM_ENTER_CRITICAL(void)
{
    while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
  		vTaskDelay( ( TickType_t)( 10 ) );   
}
//------------------------------------------------------------------------------
void SYSTEM_EXIT_CRITICAL(void)
{
    xSemaphoreGive( sem_SYSVars );
}
//------------------------------------------------------------------------------
bool u_config_debug( char *tipo, char *valor)
{
    /*
     * Configura las flags de debug para ayudar a visualizar los problemas
     * ainput,counter,modbus,piloto,wan, consigna
     */
    
    if (!strcmp_P( strupr(tipo), PSTR("MODBUS")) ) {
        if (!strcmp_P( strupr(valor), PSTR("TRUE")) ) {
            modbus_slave_config_debug(true);
            return(true);
        }
        
        if (!strcmp_P( strupr(valor), PSTR("FALSE")) ) {
            modbus_slave_config_debug(false);
            return(true);
        }
    }
        
    return(false);
    
}
//------------------------------------------------------------------------------
void u_print_tasks_running(void)
{
    
    xprintf_P(PSTR(" task running:"));
    
    if ( tk_running[TK_CMD] ) {
        xprintf_P(PSTR(" cmd"));
    }
    

    if ( tk_running[TK_RS485RX] ) {
        xprintf_P(PSTR(" rs485rx"));
    }

    if ( tk_running[TK_SYS] ) {
        xprintf_P(PSTR(" sys"));
    }

    if ( tk_running[TK_LOG] ) {
        xprintf_P(PSTR(" log"));
    }
    
    xprintf_P(PSTR("\r\n"));
    
}
//------------------------------------------------------------------------------
void u_print_watchdogs(void)
{
    
    xprintf_P(PSTR(" watchdogs:"));
    
    if ( tk_watchdog[TK_CMD] ) {
        xprintf_P(PSTR(" cmd"));
    }
    
    if ( tk_watchdog[TK_RS485RX] ) {
        xprintf_P(PSTR(" rs485rx"));
    }

    if ( tk_watchdog[TK_SYS] ) {
        xprintf_P(PSTR(" sys"));
    }
    
    if ( tk_watchdog[TK_LOG] ) {
        xprintf_P(PSTR(" log"));
    }
    
    xprintf_P(PSTR("\r\n"));
    
}
//------------------------------------------------------------------------------
void RS485_AWAKE(void)
{
    
    /*
     * Si esta despierto no hago nada
     */
    //taskENTER_CRITICAL();
    if  ( rs485_awake ==  true ) {
        return;
        
    } else {
        rs485_awake = true;
        // Despierto a la tarea.
        //xTaskNotifyGive( xHandle_tkRS485RX );
        xTaskNotify( xHandle_tkRS485RX,
                       0x01,
                       eSetBits);
    
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
    }
    //taskEXIT_CRITICAL();
    

    
}
//------------------------------------------------------------------------------
void RS485_SLEEP(void)
{
    //taskENTER_CRITICAL();
    rs485_awake = false;
    //taskEXIT_CRITICAL();
    
}
//------------------------------------------------------------------------------
void u_config_termsense(void)
{
    /*
     * Configuro el pin TERMSENSE para que sea input, sin pull-up
     */
    cli();
    CONFIG_TERM_SENSE();
    PORTA.PIN3CTRL |= PORT_PULLUPEN_bm;
    sei();
    
}
//------------------------------------------------------------------------------
uint8_t u_read_termsense(void)
{
    return  ( ( TERM_SENSE_PORT.IN & TERM_SENSE_PIN_bm ) >> TERM_SENSE_PIN);
}
// -----------------------------------------------------------------------------
void pin_awake_config(void)
{
    /*
     * Configuramos el pin para que sea entrada e interumpa en c/flanco.
     * 
     */
    
    // Es una input
    CONFIG_AWAKE_SENSE();
    
    // Habilito a interrumpir, pullup, flanco de bajada.
    cli();
    PORTC.PIN1CTRL |= PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
    sei();
    
}
// -----------------------------------------------------------------------------
#define PC1_INTERRUPT               ( PORTC.INTFLAGS & PIN1_bm )
#define PC1_CLEAR_INTERRUPT_FLAG    ( PORTC.INTFLAGS &= PIN1_bm )


ISR(PORTC_PORT_vect)
{
BaseType_t xHigherPriorityTaskWoken;

    
    // Borro las flags.
    if (PC1_INTERRUPT) {
        
        // Si el pulso fue de subida
        if ( READ_AWAKE_SENSE() == 1) {
            if ( ! rs485_awake ) {
                rs485_awake = true;
                // Despierto a la tarea.
                xHigherPriorityTaskWoken = pdFALSE;
                xTaskNotifyFromISR( xHandle_tkRS485RX,0x01, eSetBits, &xHigherPriorityTaskWoken);
            }
            
        } else {
            // Fue de bajada
            rs485_awake = false;
        }
                     
        // Se borra la flag de interrupcion para habilitarla de nuevo
        // Si no la borro antes de salir, se vuelve a generar la int.
        PC1_CLEAR_INTERRUPT_FLAG;
    }

}
// -----------------------------------------------------------------------------
