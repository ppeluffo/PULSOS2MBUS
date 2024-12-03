
/*
 * File:   spxR2_main.c
 * Author: pablo
 *
 */

#include "PULSE2MBUS.h"

/*

FUSES = {
	.WDTCFG = 0x00, // WDTCFG {PERIOD=OFF, WINDOW=OFF}
	.BODCFG = 0x00, // BODCFG {SLEEP=DISABLE, ACTIVE=DISABLE, SAMPFREQ=128Hz, LVL=BODLEVEL0}
	.OSCCFG = 0xF8, // OSCCFG {CLKSEL=OSCHF}
	.SYSCFG0 = 0xD2, // SYSCFG0 {EESAVE=CLEAR, RSTPINCFG=GPIO, CRCSEL=CRC16, CRCSRC=NOCRC}
	.SYSCFG1 = 0xF8, // SYSCFG1 {SUT=0MS}
	.CODESIZE = 0x00, // CODESIZE {CODESIZE=User range:  0x0 - 0xFF}
	.BOOTSIZE = 0x00, // BOOTSIZE {BOOTSIZE=User range:  0x0 - 0xFF}
};

LOCKBITS = 0x5CC5C55C; // {KEY=NOLOCK}
*/


FUSES = {
	.WDTCFG = 0x0B, // WDTCFG {PERIOD=8KCLK, WINDOW=OFF}
	.BODCFG = 0x00, // BODCFG {SLEEP=DISABLE, ACTIVE=DISABLE, SAMPFREQ=128Hz, LVL=BODLEVEL0}
	.OSCCFG = 0xF8, // OSCCFG {CLKSEL=OSCHF}
	.SYSCFG0 = 0xD2, // SYSCFG0 {EESAVE=CLEAR, RSTPINCFG=GPIO, CRCSEL=CRC16, CRCSRC=NOCRC}
	.SYSCFG1 = 0xF8, // SYSCFG1 {SUT=0MS}
	.CODESIZE = 0x00, // CODESIZE {CODESIZE=User range:  0x0 - 0xFF}
	.BOOTSIZE = 0x00, // BOOTSIZE {BOOTSIZE=User range:  0x0 - 0xFF}
};

LOCKBITS = 0x5CC5C55C; // {KEY=NOLOCK}

//------------------------------------------------------------------------------
int main(void) {

uint8_t i;
    
// Leo la causa del reset para trasmitirla en el init.
	wdg_resetCause = RSTCTRL.RSTFR;
	RSTCTRL.RSTFR = 0xFF;
    
    // Init out of rtos !!!
    system_init();
    
    frtos_open(fdTERM, 9600 );
    frtos_open(fdRS485, 9600 );
    frtos_open(fdNVM, 0 );
    
    sem_SYSVars = xSemaphoreCreateMutexStatic( &SYSVARS_xMutexBuffer );

    counter_init_outofrtos(&xHandle_tkLog);
    
    // Inicializo el vector de tareas activas
    for (i=0; i< RUNNING_TASKS; i++) {
        tk_running[i] = false;
    }

    // Inicializo el vector de watchdogs
    for (i=0; i< RUNNING_TASKS; i++) {
        tk_watchdog[i] = false;
    }
    
    starting_flag = false;
    
    xHandle_tkCtl = xTaskCreateStatic( tkCtl, "CTL", tkCtl_STACK_SIZE, (void *)1, tkCtl_TASK_PRIORITY, tkCtl_Buffer, &tkCtl_Buffer_Ptr );
    xHandle_tkCmd = xTaskCreateStatic( tkCmd, "CMD", tkCmd_STACK_SIZE, (void *)1, tkCmd_TASK_PRIORITY, tkCmd_Buffer, &tkCmd_Buffer_Ptr );
    xHandle_tkSys = xTaskCreateStatic( tkSys, "SYS", tkSys_STACK_SIZE, (void *)1, tkSys_TASK_PRIORITY, tkSys_Buffer, &tkSys_Buffer_Ptr );
    xHandle_tkLog = xTaskCreateStatic( tkLog, "LOG", tkLog_STACK_SIZE, (void *)1, tkLog_TASK_PRIORITY, tkLog_Buffer, &tkLog_Buffer_Ptr );
    xHandle_tkRS485RX = xTaskCreateStatic( tkRS485RX, "RS485", tkRS485RX_STACK_SIZE, (void *)1, tkRS485RX_TASK_PRIORITY, tkRS485RX_Buffer, &tkRS485RX_Buffer_Ptr );
 
    /* Arranco el RTOS. */
	vTaskStartScheduler();
  
	// En caso de panico, aqui terminamos.
	exit (1);
    
}
//------------------------------------------------------------------------------
/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
//------------------------------------------------------------------------------------
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

	/* Pass out a pointer to the StaticTask_t structure in which the Timer
	task's state will be stored. */
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

	/* Pass out the array that will be used as the Timer task's stack. */
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;

	/* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
//-----------------------------------------------------------------------------


