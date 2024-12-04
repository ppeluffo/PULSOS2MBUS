
#include "PULSE2MBUS.h"
#include "frtos_cmd.h"

static void cmdClsFunction(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdStatusFunction(void);
static void cmdConfigFunction(void);
static void cmdTestFunction(void);

static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void );

//uint16_t uxHighWaterMark;

#define CMD_TIMER_AWAKE 30000

typedef enum { CMD_AWAKE=0, CMD_SLEEP } t_cmd_pwrmode;

t_cmd_pwrmode cmd_pwrmode;
int32_t cmd_state_timer;

//------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

	// Esta es la primer tarea que arranca.

( void ) pvParameters;
uint8_t c = 0;

 //   uxHighWaterMark = SPYuxTaskGetStackHighWaterMark( NULL );
    
    while ( ! starting_flag )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

    // Marco la tarea activa
    SYSTEM_ENTER_CRITICAL();
    tk_running[TK_CMD] = true;
    SYSTEM_EXIT_CRITICAL();
             
	//vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );

 //   xprintf_P(PSTR("STACK::cmd_hwm 1 = %d\r\n"),uxHighWaterMark );

    FRTOS_CMD_init();

    FRTOS_CMD_register( "cls", cmdClsFunction );
	FRTOS_CMD_register( "help", cmdHelpFunction );
    FRTOS_CMD_register( "reset", cmdResetFunction );
    FRTOS_CMD_register( "status", cmdStatusFunction );
    FRTOS_CMD_register( "config", cmdConfigFunction );
    FRTOS_CMD_register( "test", cmdTestFunction );
    
    xprintf_P(PSTR("Starting tkCmd..\r\n" ));
    xprintf_P(PSTR("Spymovil %s %s %s %s \r\n") , HW_MODELO, FRTOS_VERSION, FW_REV, FW_DATE);
      
 //   uxHighWaterMark = SPYuxTaskGetStackHighWaterMark( NULL );
 //   xprintf_P(PSTR("STACK::cmd_hwm 2 = %d\r\n"),uxHighWaterMark );
       
    for(;;)
    {
        u_kick_wdt(TK_CMD);
        while ( xgetc( (char *)&c ) == 1 ) {
            FRTOS_CMD_process(c);
        }
        
        //vTaskDelay( ( TickType_t)( 60000 / portTICK_PERIOD_MS ) );
        //vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
        
        if ( u_read_termsense() == 0 ) {
            vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
        } else {
            vTaskDelay( ( TickType_t)( 60000 / portTICK_PERIOD_MS ) );
        } 
                           
    }      
}
//------------------------------------------------------------------------------
void __tkCmd(void * pvParameters)
{

	// Esta es la primer tarea que arranca.

( void ) pvParameters;
uint8_t c = 0;

 //   uxHighWaterMark = SPYuxTaskGetStackHighWaterMark( NULL );
    
    while ( ! starting_flag )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

    // Marco la tarea activa
    SYSTEM_ENTER_CRITICAL();
    tk_running[TK_CMD] = true;
    SYSTEM_EXIT_CRITICAL();
             
	//vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );

 //   xprintf_P(PSTR("STACK::cmd_hwm 1 = %d\r\n"),uxHighWaterMark );

    FRTOS_CMD_init();

    FRTOS_CMD_register( "cls", cmdClsFunction );
	FRTOS_CMD_register( "help", cmdHelpFunction );
    FRTOS_CMD_register( "reset", cmdResetFunction );
    FRTOS_CMD_register( "status", cmdStatusFunction );
    FRTOS_CMD_register( "config", cmdConfigFunction );
    FRTOS_CMD_register( "test", cmdTestFunction );
    
    xprintf_P(PSTR("Starting tkCmd..\r\n" ));
    xprintf_P(PSTR("Spymovil %s %s %s %s \r\n") , HW_MODELO, FRTOS_VERSION, FW_REV, FW_DATE);
      
 //   uxHighWaterMark = SPYuxTaskGetStackHighWaterMark( NULL );
 //   xprintf_P(PSTR("STACK::cmd_hwm 2 = %d\r\n"),uxHighWaterMark );
       
    cmd_pwrmode = CMD_AWAKE;
    cmd_state_timer = CMD_TIMER_AWAKE;

    for(;;)
    {
        u_kick_wdt(TK_CMD);
        
        switch (cmd_pwrmode) {
            
        case CMD_AWAKE:
 
            cmd_state_timer = CMD_TIMER_AWAKE;
            c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
            while(cmd_state_timer > 0) {
                cmd_state_timer -= 10;
                // xgetc espera 10ms !!
                while ( xgetc( (char *)&c ) == 1 ) {
                    FRTOS_CMD_process(c);
                    cmd_state_timer = CMD_TIMER_AWAKE;
                }
            }
            
            // Expiro el timer sin recibir datos: veo si entro en tickless...
            //xprintf_P(PSTR("tkCmd awake check termsense (%d)..\r\n" ), u_read_termsense() );
            vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
            if ( u_read_termsense() == 1 ) {
                xprintf_P(PSTR("tkCmd going to sleep..\r\n" ));
                vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
                //cmd_disable_TERM_uart();
                cmd_pwrmode = CMD_SLEEP;                
            } 
            
            break;
            
        case CMD_SLEEP:
            
            // Duermo 30s para que la tarea entre en modo tickless.
            vTaskDelay( ( TickType_t)(30000 / portTICK_PERIOD_MS ) );
            
            //xprintf_P(PSTR("tkCmd sleep check termsense (%d)..\r\n" ), u_read_termsense() );
            if ( u_read_termsense() == 0 ) {
                //cmd_enable_TERM_uart();
                vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
                cmd_pwrmode = CMD_AWAKE;     
                xprintf_P(PSTR("tkCmd awake..(%d)\r\n" ), u_read_termsense() );
            } 
            break;
        }                     
    }      
}
//------------------------------------------------------------------------------
static void cmdTestFunction(void)
{

    FRTOS_CMD_makeArgv();

    // AWAKE
    if (!strcmp_P( strupr(argv[1]), PSTR("AWAKE"))  ) {
        xprintf_P(PSTR("AWAKE PIN=%d\r\n"), READ_AWAKE_SENSE());
        return;
    }
    
    // TERMSENSE
    if (!strcmp_P( strupr(argv[1]), PSTR("TSENSE"))  ) {
        xprintf_P(PSTR("TERMSENSE=%d\r\n"), u_read_termsense());
        return;
    }
     
    // RS485
    if (!strcmp_P( strupr(argv[1]), PSTR("RS485"))  ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("WRITE"))  ) {
            //SET_RTS_RS485();
            vTaskDelay( ( TickType_t)( 5 ) );   
            xfprintf_P( fdRS485, PSTR("The quick brown fox jumps over the lazy dog \r\n"));
            vTaskDelay( ( TickType_t)( 2 ) );
            // RTS OFF: Habilita la recepcion del chip
            //CLEAR_RTS_RS485();
            pv_snprintfP_OK();
            return;
        }
        
        
        if (!strcmp_P( strupr(argv[2]), PSTR("READ"))  ) {
            modbus_debug_read();
            //modbus_slave_print_rx_buffer();
            //xprintf_P(PSTR("COUNT=%d\r\n"), modbus_count());
            pv_snprintfP_OK();
            return;
        }
        
        
        pv_snprintfP_ERR();
        return;  
    }
              
    // RTS {on|off}
    if (!strcmp_P( strupr(argv[1]), PSTR("RTS"))  ) {
               
        if (!strcmp_P( strupr(argv[2]), PSTR("ON"))  ) {
            SET_RTS_RS485();
            pv_snprintfP_OK();
            return;
        }        

        if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))  ) {
            CLEAR_RTS_RS485();
            pv_snprintfP_OK();
            return;
        } 
        
        pv_snprintfP_ERR();
        return;
    }
    
    if (!strcmp_P( strupr(argv[1]), PSTR("CPIN"))  ) {
        xprintf_P(PSTR("CNTpin=%d\r\n"), counter_read_pin());
        pv_snprintfP_OK();
        return;
    }

    pv_snprintfP_ERR();
    return;
       
}
//------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

    FRTOS_CMD_makeArgv();
        
    if ( !strcmp_P( strupr(argv[1]), PSTR("CONFIG"))) {
		xprintf_P( PSTR("-config:\r\n"));
        xprintf_P( PSTR("  timerpoll,magPP\r\n"));
        xprintf_P( PSTR("  modbus address\r\n"));
        xprintf_P( PSTR("  default,save,load\r\n"));
        xprintf_P( PSTR("  debug {modbus} {true/false}\r\n"));

        
    	// HELP RESET
	} else if (!strcmp_P( strupr(argv[1]), PSTR("RESET"))) {
		xprintf_P( PSTR("-reset\r\n"));
		return;
        
    } else if (!strcmp_P( strupr(argv[1]), PSTR("TEST"))) {
		xprintf_P( PSTR("-test\r\n"));
        xprintf_P( PSTR("  rts {on|off}\r\n"));
        xprintf_P( PSTR("  cpin, awake\r\n"));
        xprintf_P( PSTR("  rs485 write,read\r\n"));
        return;
        
    }  else {
        // HELP GENERAL
        xprintf("Available commands are:\r\n");
        xprintf("-cls\r\n");
        xprintf("-help\r\n");
        xprintf("-status\r\n");
        xprintf("-reset\r\n");
        xprintf("-config...\r\n");
    }
   
	xprintf("Exit help \r\n");

}
//------------------------------------------------------------------------------
static void cmdClsFunction(void)
{
	// ESC [ 2 J
	xprintf("\x1B[2J\0");
}
//------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
    
    xprintf("Reset..\r\n");
    reset();
}
//------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

    // https://stackoverflow.com/questions/12844117/printing-defined-constants


    xprintf("Spymovil %s %s TYPE=%s, VER=%s %s \r\n" , HW_MODELO, FRTOS_VERSION, FW_TYPE, FW_REV, FW_DATE);
      
    xprintf_P(PSTR("Config:\r\n"));
    xprintf_P(PSTR(" nvmid: %s\r\n"), NVM_signature2str());       
    xprintf_P(PSTR(" reset cause=0x%02x\r\n"), wdg_resetCause );
    //
    xprintf_P(PSTR(" timerpoll=%d\r\n"), counter_conf.timerpoll);
    xprintf_P(PSTR(" magpp=%0.3f\r\n"), counter_conf.magpp);
    xprintf_P(PSTR(" modbus addr= 0x%02x\r\n"), systemConf.modbus_address);
    if ( modbus_debug ) {
        xprintf_P(PSTR(" modbus debug = TRUE\r\n"));
    } else {
        xprintf_P(PSTR(" modbus debug = FALSE\r\n"));
    }
    
    u_print_tasks_running();
    u_print_watchdogs();
    
    xprintf_P(PSTR("Stats: ppm=%d, pph=%lu\r\n"), contador.pulsosXmin, contador.pulsosXhora);


}
//------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{
    
    FRTOS_CMD_makeArgv();
        
    // MODBUS:
	if ( strcmp_P ( strupr( argv[1]), PSTR("MODBUS")) == 0 ) {
        
        // addr 
        if ( strcmp_P ( strupr( argv[2]), PSTR("ADDRESS")) == 0 ) {
            systemConf.modbus_address = atoi(argv[3]);
            mbus_local_address = systemConf.modbus_address;
            pv_snprintfP_OK();
            return;
        }
    }
     
    if ( strcmp_P ( strupr( argv[1]), PSTR("MAGPP")) == 0 ) {
        counter_config_magpp(argv[2]);
        pv_snprintfP_OK();
        return;
    }
        
    if ( strcmp_P ( strupr( argv[1]), PSTR("TIMERPOLL")) == 0 ) {
        counter_config_timerpoll(argv[2]);
        pv_snprintfP_OK();
        return;
    }
     
	// SAVE
	// config save
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE"))) {       
		u_save_config_in_NVM();
		pv_snprintfP_OK();
		return;
	}
    
    // LOAD
	// config load
	if (!strcmp_P( strupr(argv[1]), PSTR("LOAD"))) {
		u_load_config_from_NVM();
		pv_snprintfP_OK();
		return;
	}
    
    // DEFAULT
	// config default
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT"))) {
        u_config_default();
		pv_snprintfP_OK();
		return;
	}
    
        // DEBUG
    // config debug (modbus) (true,false)
    if (!strcmp_P( strupr(argv[1]), PSTR("DEBUG")) ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("MODBUS")) ) {
            if (!strcmp_P( strupr(argv[3]), PSTR("TRUE")) ) {
                modbus_slave_config_debug(true);
                pv_snprintfP_OK();
                return;
            }
            if (!strcmp_P( strupr(argv[3]), PSTR("FALSE")) ) {
                modbus_slave_config_debug(false);
                pv_snprintfP_OK();
                return;
            }            
        }
        pv_snprintfP_ERR();
        return;
    }
           
    // CMD NOT FOUND
	xprintf("ERROR\r\nCMD NOT DEFINED\r\n\0");
	return;
 
}
//------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	xprintf("ok\r\n\0");
}
//------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	xprintf("error\r\n\0");
}
//------------------------------------------------------------------------------
