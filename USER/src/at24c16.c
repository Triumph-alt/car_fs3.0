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


/*--------------------------------------------------------------
 * @brief   上电后从AT24C16 EEPROM读取参数
 *          1. max_value[0..6]  (Page0  Addr0~13)
 *          2. SpeedPID kp,ki,kd (Page1  Addr0~11)
 *          3. TurnPID  kp,ki,kd (Page2  Addr0~11)
 *-------------------------------------------------------------*/

/* 辅助函数: 读取4字节并转换为float */
float eeprom_read_float(uint8_t base_page, uint8_t base_offset)
{
    union { float f; uint8_t b[4]; } _u;

    uint8_t  k;
    uint16_t off;
    uint8_t  pg, ad;

    for(k = 0; k < 4; k++)
    {
        off = base_offset + k;
        pg  = (uint8_t)(base_page + (off / 16));
        ad  = (uint8_t)(off % 16);
        _u.b[k] = at24c16_read_byte(pg, ad);
    }

    return _u.f;
}

void load_parameters_from_eeprom(void)
{
    uint8_t i;

    /* 1. 读取 max_value 数组 */
    for(i = 0; i < 7; i++)
    {
        max_value[i] = at24c16_read_twobytes(0, (uint8_t)(i * 2));
    }

    /* 2. 读取 SpeedPID 参数 */
    SpeedPID.kp = eeprom_read_float(1, 0);
    SpeedPID.ki = eeprom_read_float(1, 4);
    SpeedPID.kd = eeprom_read_float(1, 8);

    /* 3. 读取 TurnPID 参数 */
    TurnPID.kp = eeprom_read_float(2, 0);
    TurnPID.ki = eeprom_read_float(2, 4);
    TurnPID.kd = eeprom_read_float(2, 8);
}

//----------------------------------------------------------------------------- 
// @brief    保存 max_value 数组与 PID 参数到 AT24C16 EEPROM
// @note     采用如下地址映射：
//           Page0  Addr0~13   -> max_value[0..6]   (每个 uint16 占 2 字节)
//           Page1  Addr0~11   -> SpeedPID  kp,ki,kd (每个 float 占 4 字节)
//           Page2  Addr0~11   -> TurnPID   kp,ki,kd (每个 float 占 4 字节)
//           如需调整映射，请同步修改读取函数
//-----------------------------------------------------------------------------
void save_parameters_to_eeprom(void)
{
    uint8_t i;
    /* 1. 写入 max_value (uint16) */
    for(i = 0; i < 7; i++)
    {
        /* Page0 起始地址按 2*i */
        at24c16_write_twobytes(0, (uint8_t)(i * 2), max_value[i]);
    }

    /* 工具宏：将 float 拆分为 4 个字节并写入 */
    #define WRITE_FLOAT_TO_EEPROM(base_page, base_offset, fval)                 \
        do{                                                                    \
            union { float f; uint8_t b[4]; } _u;                               \
            uint8_t  _k;                                                       \
            uint16_t _off;                                                     \
            uint8_t  _pg, _ad;                                                 \
            _u.f = (fval);                                                     \
            for(_k = 0; _k < 4; _k++)                                          \
            {                                                                  \
                _off = (base_offset) + _k;                                     \
                _pg  = (uint8_t)((base_page) + (_off / 16));                  \
                _ad  = (uint8_t)(_off % 16);                                   \
                at24c16_write_byte(_pg, _ad, _u.b[_k]);                        \
            }                                                                  \
        }while(0)

    /* 2. 写入 SpeedPID 参数到 Page1 起始偏移 0 */
    WRITE_FLOAT_TO_EEPROM(1, 0, SpeedPID.kp);
    WRITE_FLOAT_TO_EEPROM(1, 4, SpeedPID.ki);
    WRITE_FLOAT_TO_EEPROM(1, 8, SpeedPID.kd);

    /* 3. 写入 TurnPID 参数到 Page2 起始偏移 0 */
    WRITE_FLOAT_TO_EEPROM(2, 0, TurnPID.kp);
    WRITE_FLOAT_TO_EEPROM(2, 4, TurnPID.ki);
    WRITE_FLOAT_TO_EEPROM(2, 8, TurnPID.kd);

    #undef WRITE_FLOAT_TO_EEPROM
}
