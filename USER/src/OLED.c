#include "headfile.h"
#include "OLED_Font.h"
#include "zf_iic.h"

/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
	iic_write_reg(0x3C, 0x00, Command);
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
	iic_write_reg(0x3C, 0x40, Data);
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置X位置低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void oled_clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void oled_show_char(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	
//	OLED_SetCursor((uint8_t)((Line - 1) * 2), (uint8_t)((Column - 1) * 8));		//设置光标位置在上半部分
//	
//	for (i = 0; i < 8; i++)
//	{
//		OLED_WriteData(OLED_F8x16[Char - ' '][i]);			//显示上半部分内容
//	}
	
//	OLED_SetCursor((uint8_t)((Line - 1) * 2 + 1), (uint8_t)((Column - 1) * 8));	//设置光标位置在下半部分
//	
//	for (i = 0; i < 8; i++)
//	{
//		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);		//显示下半部分内容
//	}
	
	OLED_SetCursor((uint8_t)(Line - 1), (uint8_t)((Column - 1) * 6));		//设置光标位置在上半部分
	
	for (i = 0; i < 6; i++)
	{
		OLED_WriteData(OLED_F6x8[Char - ' '][i]);			//显示上半部分内容
	}
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void oled_show_string(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		oled_show_char(Line, (uint8_t)(Column + i), String[i]);
	}
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~4294967295
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void oled_show_num(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		oled_show_char(Line, (uint8_t)(Column + i), (uint8_t)(Number / OLED_Pow(10, Length - i - 1) % 10 + '0'));
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void oled_show_signednum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		oled_show_char(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		oled_show_char(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		oled_show_char(Line, (uint8_t)(Column + i + 1), (uint8_t)(Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0'));
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void oled_show_hexnum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			oled_show_char(Line, (uint8_t)(Column + i), (uint8_t)(SingleNumber + '0'));
		}
		else
		{
			oled_show_char(Line, (uint8_t)(Column + i), (uint8_t)(SingleNumber - 10 + 'A'));
		}
	}
}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void oled_show_binnum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		oled_show_char(Line, (uint8_t)(Column + i), (uint8_t)(Number / OLED_Pow(2, Length - i - 1) % 2 + '0'));
	}
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void oled_init(void)
{
//	uint32_t i, j;
//	
//	for (i = 0; i < 1000; i++)			//上电延时
//	{
//		for (j = 0; j < 1000; j++);
//	}
	
	OLED_WriteCommand(0xAE);	//关闭显示
	
	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);	//设置显示开始行
	
	OLED_WriteCommand(0xA0);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand(0xC0);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	//开启显示
		
	oled_clear();				//OLED清屏
}

// 新增：OLED显示浮点数，保留两位小数，格式±xxx.xx
typedef union { float f; uint32_t u32; } _f32u32;
void oled_show_float(uint8_t Line, uint8_t Column, float Number)
{
    /* 该函数在OLED上以固定格式显示带符号的浮点数，保留两位小数。
       显示格式示例："+123.45" 或 "-  0.25"，总长度7个字符。
       参数说明：
         Line   - 起始行位置 (1~8)
         Column - 起始列位置 (1~21)，指示要显示的第一个字符
         Number - 要显示的浮点数
    */
    int32_t int_part;
    int32_t dec_part;

    // 处理符号
    if (Number < 0)
    {
        oled_show_char(Line, Column, '-');
        Number = -Number;
    }
    else
    {
        oled_show_char(Line, Column, '+');
    }

    // 拆分整数和小数部分（两位小数，四舍五入）
    int_part = (int32_t)Number;
    dec_part = (int32_t)((Number - int_part) * 100 + 0.5f);

    // 进位处理，例如 1.999 -> 2.00
    if (dec_part >= 100)
    {
        dec_part -= 100;
        int_part += 1;
    }

    // 显示整数部分（3位，不足补0）
    oled_show_num(Line, (uint8_t)(Column + 1), (uint32_t)int_part, 3);

    // 显示小数点
    oled_show_char(Line, (uint8_t)(Column + 4), '.');

    // 显示小数部分（2位，不足补0）
    oled_show_num(Line, (uint8_t)(Column + 5), (uint32_t)dec_part, 2);
}
