/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#ifndef F_CPU
#define F_CPU 24000000
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"
#include "portable.h"
#include "portmacro.h"
#include "protected_io.h"
#include "ccp.h"

#include <avr/wdt.h> 
#include <avr/pgmspace.h>
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "frtos-io.h"
#include "xprintf.h"
#include "xgetc.h"
#include "nvm.h"
#include "led.h"
#include "pines.h"
#include "linearBuffer.h"
#include "contadores.h"
#include "modbus_slave.h"
#include "bits.h"
#include "pines.h"

#define FW_REV "1.0.0"
#define FW_DATE "@ 20241028"
#define HW_MODELO "PULSOS2MBUS FRTOS R001 HW:AVR128DA64"
#define FRTOS_VERSION "FreeRTOS V202111.00"
#define FW_TYPE "PULSOS2MBUS"

#define SYSMAINCLK 24

#define tkCtl_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )
#define tkRS485RX_TASK_PRIORITY ( tskIDLE_PRIORITY + 1 )

#define tkCtl_STACK_SIZE		384
#define tkCmd_STACK_SIZE		512
#define tkRS485RX_STACK_SIZE	384

StaticTask_t tkCtl_Buffer_Ptr;
StackType_t tkCtl_Buffer [tkCtl_STACK_SIZE];

StaticTask_t tkCmd_Buffer_Ptr;
StackType_t tkCmd_Buffer [tkCmd_STACK_SIZE];

StaticTask_t tkRS485RX_Buffer_Ptr;
StackType_t tkRS485RX_Buffer [tkRS485RX_STACK_SIZE];

SemaphoreHandle_t sem_SYSVars;
StaticSemaphore_t SYSVARS_xMutexBuffer;

#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

TaskHandle_t xHandle_tkCtl, xHandle_tkCmd, xHandle_tkRS485RX;

void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);
void tkRS485RX(void * pvParameters);

bool starting_flag;

// Estructura que tiene la configuracion del sistema
struct {

    uint8_t modbus_address;
} systemConf;

struct {
    uint16_t pulsos;
    float caudal;
    float duracion_pulso;
    uint32_t ticks_now;
    uint32_t pulsoWidth_ticks;    
} systemVars;


void system_init();
void reset(void);
bool u_config_timerpoll ( char *s_timerpoll );
void u_config_default(void);
bool u_save_config_in_NVM(void);
bool u_load_config_from_NVM(void);
void SYSTEM_ENTER_CRITICAL(void);
void SYSTEM_EXIT_CRITICAL(void);
bool u_config_debug( char *tipo, char *valor);
void u_print_tasks_running(void);
void u_config_termsense(void);
uint8_t u_read_termsense(void);

// Mensajes entre tareas
#define SIGNAL_FRAME_READY		0x01

// Task running & watchdogs
#define RUNNING_TASKS   2
typedef enum { TK_CMD = 0, TK_RS485RX } t_wdg_ids;
bool tk_running[RUNNING_TASKS];
bool tk_watchdog[RUNNING_TASKS];

void u_kick_wdt( t_wdg_ids wdg_id);
void u_print_watchdogs(void);

bool rs485_awake;
void RS485_AWAKE(void);
void RS485_SLEEP(void);

uint8_t wdg_resetCause;

#endif	/* XC_HEADER_TEMPLATE_H */

