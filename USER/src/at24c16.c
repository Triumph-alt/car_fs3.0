#include "at24c16.h"

void at24c16_write_byte(uint8_t page, uint8_t addr, uint8_t w_data)
{
	uint8_t ret = 1;
	uint8_t device_addr = 0x00, data_addr = 0x00;
	
	/* 计算设备地址和字节地址 */
	device_addr = ((0x0A << 3) | ((page >> 4) & (0x07)));
	data_addr = (((page & 0x0F) << 4) | (addr & 0x0F));
	
	ret = iic_write_reg(device_addr, data_addr, w_data);

	delay_ms(5);
}

uint8_t at24c16_read_byte(uint8_t page, uint8_t addr)
{
	uint8_t ret = 1;
	uint8_t device_addr = 0x00, data_addr = 0x00;
	uint8_t r_data = 0;
	
	/* 计算设备地址和字节地址 */
	device_addr = ((0x0A << 3) | ((page >> 4) & (0x07)));
	data_addr = (((page & 0x0F) << 4) | (addr & 0x0F));
	
	ret = iic_read_reg(device_addr, data_addr, &r_data);
	
	return r_data;
}

void at24c16_write_twobytes(uint8_t page, uint8_t addr, uint16_t w_data)
{
	uint8_t h_data = 0, l_data = 0;
	
	/* 拆成高低8位分别写入 */
	h_data = w_data >> 8;
	l_data = w_data & 0xff;
	
	at24c16_write_byte(page, addr, h_data);
	at24c16_write_byte(page, (uint8_t)(addr + 1), l_data);
}

uint16_t at24c16_read_twobytes(uint8_t page, uint8_t addr)
{
	uint8_t h_data = 0, l_data = 0;
	uint16_t r_data = 0;
	
	h_data = at24c16_read_byte(page, addr);
	l_data = at24c16_read_byte(page, (uint8_t)(addr + 1));
	
	r_data = ((h_data << 8) | l_data);
	
	return r_data;
}
