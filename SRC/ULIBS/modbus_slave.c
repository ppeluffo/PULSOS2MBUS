
#include "modbus_slave.h"

bool modbus_debug;
uint8_t mbus_fsm_state;
uint8_t mbus_local_address;
modbus_slave_CONTROL_BLOCK_t mbus_cb;
counter_value_t contador;

//------------------------------------------------------------------------------
void modbus_slave_config_debug(bool debug)
{
    modbus_debug = debug;
}
//------------------------------------------------------------------------------
void modbus_slave_fsm_init(void)
{
    mbus_fsm_state = FSM_IDLE;
    
    lBchar_CreateStatic ( &mbslave_rx_lbuffer, (char *)mbslave_rx_array, MBSLAVE_RX_BUFFER_SIZE );
    lBchar_Flush(&mbslave_rx_lbuffer);

}
//------------------------------------------------------------------------------
void modbus_slave_fsm(char c)
{
      
    // Almaceno.
    if ( ! lBchar_Put(&mbslave_rx_lbuffer, c) ) {
        lBchar_Flush(&mbslave_rx_lbuffer);
        return;
    }
    
    // Ejecuto.
    switch(mbus_fsm_state) {
        
        case FSM_IDLE:
            // Estando en idle, el primer caracter debe ser la local address
            // si el frame es para mi
            // Borro el buffer y guardo el byte.
            if ( c == mbus_local_address ) {
                // Inicializo
                lBchar_Flush(&mbslave_rx_lbuffer);
                lBchar_Put(&mbslave_rx_lbuffer, c);
                mbus_fsm_state = FCN_CODE;   
            }
            break;
            
        case FCN_CODE:
            // Solo proceso los frames 3 y 6. El resto los descarto
            // Ya los guarde en el buffer al entrar.
            if ( ( c == 0x03) || ( c == 0x06 ) ) {
                mbus_fsm_state = IN_FRAME;
            } else {
                // Function codes no implementados
                mbus_fsm_state = FSM_IDLE;
                lBchar_Flush(&mbslave_rx_lbuffer);  
            }
            break;
        
        case IN_FRAME:
            // Tanto el fcn 3 o 6 son frames de 8 bytes. Controlo esto para
            // ver cuando termino el frame y debo procesarlo
            if (lBchar_GetCount(&mbslave_rx_lbuffer) == 8 ) {
                modbus_slave_process_frame();
                mbus_fsm_state = FSM_IDLE;
                lBchar_Flush(&mbslave_rx_lbuffer);   
            }
            break;
            
        default:
            mbus_fsm_state = FSM_IDLE;
            lBchar_Flush(&mbslave_rx_lbuffer);    
      
    }   
}
//------------------------------------------------------------------------------
void modbus_slave_process_frame(void)
{
    /*
     * El frame recibido esta en rx_buffer
     */
    
uint16_t crc_rcvd;
uint16_t crc_calculated;
    
    if (modbus_debug ) {
        xprintf_P(PSTR("MODBUS_fsm_process_frame\r\n"));
    }

    // Primero chequeo el CRC
    crc_rcvd = mbslave_rx_array[7] << 8 | mbslave_rx_array[6];
    crc_calculated = crc16(&mbslave_rx_array[0], 6);
    
    if ( crc_rcvd != crc_calculated ) {
        xprintf_P(PSTR("MBUS RCVD CRC ERROR\r\n"));
        xprintf_P(PSTR("CRC_RX=0x%04x, CRC_CALC=0x%04x\r\n"), crc_rcvd, crc_calculated);
        xprintf_P(PSTR("CRC1=0x%02x, CRC2=0x%02x\r\n"), mbslave_rx_array[6], mbslave_rx_array[7]);
        modbus_slave_print_rx_buffer();
        lBchar_Flush(&mbslave_rx_lbuffer);
        return;
    }
    
    // Proceso el frame
    switch(mbslave_rx_array[1]) {
        case 0x03:
            modbus_slave_process_frame03();
            break;
        default:
            xprintf_P(PSTR("MODBUS FCODE ERROR (0x%02x)\r\n"), mbslave_rx_array[1] );
            break;
    }
    
}
//------------------------------------------------------------------------------
void modbus_slave_process_frame03(void)
{
   /*
     * El frame 03 se usa para leer una direccion
     * FE 03 00 00 00 01 90 05
     */

uint16_t reg_address;
uint16_t nro_regs;
uint8_t size = 0;
uint16_t crc;

    if (modbus_debug) {
        xprintf_P(PSTR("MODBUS_fsm_process_frame03\r\n"));
        modbus_slave_print_rx_buffer();
    }
   
    reg_address = mbslave_rx_array[2] << 8 | mbslave_rx_array[3];
    nro_regs = mbslave_rx_array[4] << 8 | mbslave_rx_array[5];
    
    if (modbus_debug ) {
        xprintf_P(PSTR("reg_addr=%u, nro_regs=%u\r\n"), reg_address, nro_regs);
    }
        
    if (reg_address == 0x1) {
        
        // El reg01 el el valor del contador (float), 4 bytes
        contador = counter_read();
        mbus_cb.udata.float_value = contador.caudal;

        mbus_cb.tx_buffer[0] = mbus_local_address;     // [0xFE]
        mbus_cb.tx_buffer[1] = 0x3;                    // FCODE=0x03
        mbus_cb.tx_buffer[2] = 0x4;                    // Byte count ( 4 bytes )
        
        mbus_cb.tx_buffer[3] = mbus_cb.udata.raw_value[0];
        mbus_cb.tx_buffer[4] = mbus_cb.udata.raw_value[1];
        mbus_cb.tx_buffer[5] = mbus_cb.udata.raw_value[2];
        mbus_cb.tx_buffer[6] = mbus_cb.udata.raw_value[3];   
        
        size = 7;
        crc = crc16( mbus_cb.tx_buffer, size );
        mbus_cb.tx_buffer[size++] = (uint8_t)( crc & 0x00FF );			// CRC Low
        mbus_cb.tx_buffer[size++] = (uint8_t)( (crc & 0xFF00) >> 8 );	// CRC High
        mbus_cb.tx_size = size;
    
        modbus_slave_txmit_ADU( &mbus_cb );
    }
    
    return;
    
}
//------------------------------------------------------------------------------
void modbus_slave_txmit_ADU( modbus_slave_CONTROL_BLOCK_t *mbus_cb )
{
	// Transmite el frame modbus almcenado en mbus_cb.tx_buffer
	//
	// OJO: En MODBUS, los bytes pueden ser 0x00 y no deben ser interpretados como NULL
	// al trasmitir por lo tanto hay que usar el data_size
	// Debemos cumplir extrictos limites de tiempo por lo que primero logueo y luego
	// muestro el debug.

uint8_t i;

   
	// Transmito
	// borro buffers y espero 3.5T (9600) = 3.5ms ( START )
	vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );

    if (modbus_debug) {
        xprintf_P(PSTR("MODBUS_txmit_ADU\r\n"));
        xprintf_P( PSTR("len=%d\r\n"), mbus_cb->tx_size);
		for ( i = 0 ; i < mbus_cb->tx_size ; i++ ) {
			xprintf_P( PSTR("[0x%02X]"), mbus_cb->tx_buffer[i]);
		}
		xprintf_P( PSTR("\r\n"));
        
    }
	// La funcion xnprintf_MBUS maneja el control de flujo.
	i = xnprintf( fdRS485_MODBUS, (const char *)mbus_cb->tx_buffer, mbus_cb->tx_size );

}
//------------------------------------------------------------------------------
void modbus_slave_print_rx_buffer(void)
{
 
char *p;
uint8_t count = lBchar_GetCount(&mbslave_rx_lbuffer);
uint8_t i;

    xprintf_P(PSTR("COUNT=%d\r\n"), count  );
     
    p = lBchar_get_buffer(&mbslave_rx_lbuffer);
    //p = &rs485_rx_buffer[0];
    
    xprintf_P(PSTR("RCVD:"));
    for (i=0; i < count; i++) {
        xprintf_P(PSTR("[0x%02x]"), *p++);
    }
    
    xprintf_P(PSTR("\r\n"));
       
}
//------------------------------------------------------------------------------
uint16_t crc16( uint8_t *msg, uint8_t msg_size )
{

uint16_t crc = 0xFFFF;
uint16_t pos;
uint16_t data;
int i;

	for ( pos = 0; pos < msg_size; pos++) {
		data = (uint16_t)*msg++;
        //xprintf_P(PSTR("DATA=0x%02x\r\n"), data);
        
		crc ^= data;          // XOR byte into least sig. byte of crc

		for (i = 8; i != 0; i--) {          // Loop over each bit
			if ((crc & 0x0001) != 0) {      // If the LSB is set
				crc >>= 1;                  // Shift right and XOR 0xA001
				crc ^= 0xA001;
			} else {                            // Else LSB is not set
				crc >>= 1;                    // Just shift right
			}
		}
	}
	// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)

    //xprintf_P(PSTR("CRC=0x%04x\r\n"), crc);

	return crc;
}
//------------------------------------------------------------------------------

