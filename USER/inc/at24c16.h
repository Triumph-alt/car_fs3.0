#ifndef __AT24C16_H
#define __AT24C16_H

#include "headfile.h"

void at24c16_write_byte(uint8_t page, uint8_t addr, uint8_t w_data);
uint8_t at24c16_read_byte(uint8_t page, uint8_t addr);
void at24c16_write_twobytes(uint8_t page, uint8_t addr, uint16_t w_data);
uint16_t at24c16_read_twobytes(uint8_t page, uint8_t addr);

#endif
