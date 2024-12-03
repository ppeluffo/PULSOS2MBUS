/* 
 * File:   modbus_slave.h
 * Author: pablo
 *
 * Created on 29 de octubre de 2024, 11:17 AM
 */

#ifndef MODBUS_SLAVE_H
#define	MODBUS_SLAVE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"
#include "linearBuffer.h"
#include "xprintf.h"
#include "contadores.h"
#include "frtos-io.h"
    
typedef union {
	uint8_t raw_value[4];
	// Interpretaciones
	uint16_t u16_value;
	int16_t i16_value;
	uint32_t u32_value;
	int32_t i32_value;
	float float_value;
	char str_value[4];		// Almaceno NaN cuando hay un error.
} modbus_hold_t; // (4)

#define MBUS_TXMSG_LENGTH	16

typedef struct {
	uint8_t tx_buffer[MBUS_TXMSG_LENGTH];
	uint8_t tx_size;		// Cantidad de bytes en el txbuffer para transmitir
	modbus_hold_t udata;	// estructura para interpretar los distintos formatos de los datos.
} modbus_slave_CONTROL_BLOCK_t;

#define MBSLAVE_RX_BUFFER_SIZE 128
uint8_t mbslave_rx_array[MBSLAVE_RX_BUFFER_SIZE];
lBuffer_s mbslave_rx_lbuffer;

bool modbus_debug;
uint8_t mbus_fsm_state;
uint8_t mbus_local_address;
modbus_slave_CONTROL_BLOCK_t mbus_cb;

typedef enum { FSM_IDLE=0, FCN_CODE, IN_FRAME } fsm_states_t;

void modbus_slave_config_debug(bool debug);
void modbus_slave_fsm_init(void);
void modbus_slave_fsm(uint8_t c);
void modbus_slave_process_frame(void);
void modbus_slave_print_rx_buffer(void);
void modbus_slave_process_frame03(void);
void modbus_slave_txmit_ADU( modbus_slave_CONTROL_BLOCK_t *mbus_cb );
uint16_t crc16( uint8_t *msg, uint8_t msg_size );
uint16_t modbus_count(void);
void modbus_debug_read(void);


#ifdef	__cplusplus
}
#endif

#endif	/* MODBUS_SLAVE_H */

