
#include "bits.h"

uint8_t byte_setBit(uint8_t num, uint8_t position) {
    return num | (1 << position);
}
// Function to clear a bit
uint8_t byte_clearBit(uint8_t num, uint8_t position) {
    return num & ~(1 << position);
}
// Function to toggle a bit
uint8_t byte_toggleBit(uint8_t num, uint8_t position) {
    return num ^ (1 << position);
}
// Function to check the status of a bit
bool byte_isBitSet(uint8_t num, uint8_t position) {
    return (num & (1 << position)) != 0;
}

uint16_t word_setBit(uint16_t num, uint8_t position) {
    return num | (1 << position);
}
// Function to clear a bit
uint16_t word_clearBit(uint16_t num, uint8_t position) {
    return num & ~(1 << position);
}
// Function to toggle a bit
uint16_t word_toggleBit(uint16_t num, uint8_t position) {
    return num ^ (1 << position);
}
// Function to check the status of a bit
bool word_isBitSet(uint16_t num, uint8_t position) {
    return (num & (1 << position)) != 0;
}