#include "at24c16.h"
//----------------------------------------------------------------------------- 
// @brief    写入一个字节到 AT24C16 EEPROM
// @param    page  EEPROM 页号(0~127)，对应高地址位 A10~A4
// @param    addr  页内地址(0~15)，对应 A3~A0
// @param    w_data 要写入的 8 位数据
// @return   无
// @author   ZP
// Sample usage: at24c16_write_byte(0, 0, 0x55);  // 将 0x55 写入 0 页 0 地址
//----------------------------------------------------------------------------- 
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
//----------------------------------------------------------------------------- 
// @brief    从 AT24C16 EEPROM 读取一个字节
// @param    page  EEPROM 页号(0~127)
// @param    addr  页内地址(0~15)
// @return   读取到的数据(8 位)
// @author   ZP
// Sample usage: uint8_t data = at24c16_read_byte(0, 0);
//----------------------------------------------------------------------------- 
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
//----------------------------------------------------------------------------- 
// @brief    连续写入 2 字节到 AT24C16 EEPROM (高字节在前)
// @param    page  EEPROM 页号(0~127)
// @param    addr  起始地址(0~15)，函数内部自动写入 addr 和 addr+1
// @param    w_data 要写入的 16 位数据
// @return   无
// @author   ZP
// Sample usage: at24c16_write_twobytes(1, 10, 0x1234);
//----------------------------------------------------------------------------- 
void at24c16_write_twobytes(uint8_t page, uint8_t addr, uint16_t w_data)
{
	uint8_t h_data = 0, l_data = 0;
	
	/* 拆成高低8位分别写入 */
	h_data = w_data >> 8;
	l_data = w_data & 0xff;
	
	at24c16_write_byte(page, addr, h_data);
	at24c16_write_byte(page, (uint8_t)(addr + 1), l_data);
}
//----------------------------------------------------------------------------- 
// @brief    连续读取 2 字节数据 (高字节在前)
// @param    page  EEPROM 页号(0~127)
// @param    addr  起始地址(0~15)，函数内部自动读取 addr 和 addr+1
// @return   读取到的 16 位数据
// @author   ZP
// Sample usage: uint16_t val = at24c16_read_twobytes(1, 10);
//----------------------------------------------------------------------------- 
uint16_t at24c16_read_twobytes(uint8_t page, uint8_t addr)
{
	uint8_t h_data = 0, l_data = 0;
	uint16_t r_data = 0;
	
	h_data = at24c16_read_byte(page, addr);
	l_data = at24c16_read_byte(page, (uint8_t)(addr + 1));
	
	r_data = ((h_data << 8) | l_data);
	
	return r_data;
}
