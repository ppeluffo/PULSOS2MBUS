/* 
 * File:   bits.h
 * Author: pablo
 *
 * Created on 30 de agosto de 2024, 04:17 PM
 */

#ifndef BITS_H
#define	BITS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
    
uint8_t byte_setBit(uint8_t num, uint8_t position);
uint8_t byte_clearBit(uint8_t num, uint8_t position);
uint8_t byte_toggleBit(uint8_t num, uint8_t position);
bool byte_isBitSet(uint8_t num, uint8_t position);

uint16_t word_setBit(uint16_t num, uint8_t position);
uint16_t word_clearBit(uint16_t num, uint8_t position);
uint16_t word_toggleBit(uint16_t num, uint8_t position);
bool word_isBitSet(uint16_t num, uint8_t position);


#ifdef	__cplusplus
}
#endif

#endif	/* BITS_H */

