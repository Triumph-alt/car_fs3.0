#include "transfer.h"
#include "OLED.h"
#include "electromagnetic_tracking.h"
#include "pid.h"
#include "at24c16.h"

Key_t key[4] = {0, 0, 0, 0};
enum state car_state; 
uint8 selected_item = 0; // 当前选中的项目索引

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

void key_task(void)
{
	if (key[0].flag == 1)
	{
        enum state prev_state = car_state;   // 记录切换前状态
        if (car_state < RUNNING)
        {
            car_state++;
            oled_clear();
            selected_item = 0; // 切换状态时重置选中项

            /* 若从 PID_PARA 切换到 CHARGE，保存参数 */
            if(prev_state == PID_PARA && car_state == CHARGE)
            {
                save_parameters_to_eeprom();
            }
        }
		
		key[0].flag = 0;
	}

	if (key[1].flag == 1)
	{
        // 按键1作为选择按键
        switch (car_state)
        {
            case ELECT_PARA:
                // 在电感参数界面，最多7个选项(max_value数组)
                selected_item = (selected_item + 1) % 7;
                break;
                
            case PID_PARA:
                // 在PID参数界面，最多6个选项(SpeedPID和TurnPID的kp,ki,kd)
                selected_item = (selected_item + 1) % 6;
                break;
                
            default:
                break;
        }
		
		key[1].flag = 0;
	}

	if (key[2].flag == 1)
	{
        // 按键2作为增加值的按键
        switch (car_state)
        {
            case ELECT_PARA:
                // 增加选中的max_value值
                if (selected_item < 7)
                {
                    max_value[selected_item] += 10; // 每次增加10
                }
                break;
                
            case PID_PARA:
                // 增加选中的PID参数
                if (selected_item == 0) SpeedPID.kp += 0.1f;
                else if (selected_item == 1) SpeedPID.ki += 0.1f;
                else if (selected_item == 2) SpeedPID.kd += 0.1f;
                else if (selected_item == 3) TurnPID.kp += 0.1f;
                else if (selected_item == 4) TurnPID.ki += 0.1f;
                else if (selected_item == 5) TurnPID.kd += 0.1f;
                break;
                
            default:
                break;
        }
		
		key[2].flag = 0;
	}

	if (key[3].flag == 1)
	{
        // 按键3作为减少值的按键
        switch (car_state)
        {
            case ELECT_PARA:
                // 减少选中的max_value值
                if (selected_item < 7 && max_value[selected_item] >= 10)
                {
                    max_value[selected_item] -= 10; // 每次减少10
                }
                break;
                
            case PID_PARA:
                // 减少选中的PID参数
                if (selected_item == 0 && SpeedPID.kp >= 0.1f) SpeedPID.kp -= 0.1f;
                else if (selected_item == 1 && SpeedPID.ki >= 0.1f) SpeedPID.ki -= 0.1f;
                else if (selected_item == 2 && SpeedPID.kd >= 0.1f) SpeedPID.kd -= 0.1f;
                else if (selected_item == 3 && TurnPID.kp >= 0.1f) TurnPID.kp -= 0.1f;
                else if (selected_item == 4 && TurnPID.ki >= 0.1f) TurnPID.ki -= 0.1f;
                else if (selected_item == 5 && TurnPID.kd >= 0.1f) TurnPID.kd -= 0.1f;
                break;
                
            default:
                break;
        }
		
		key[3].flag = 0;
	}
}

/* 8行，21列*/
void display_task(void)
{
    switch (car_state)
    {
        case ELECT_PARA:
            {
                oled_show_string(1, 8, "max_v:");
                
                // 显示电感名称，选中项前添加'>'标记
                oled_show_string(2, 1, selected_item == 0 ? ">HL :" : "HL :");
                oled_show_string(3, 1, selected_item == 1 ? ">HML:" : "HML:");
                oled_show_string(4, 1, selected_item == 2 ? ">HC :" : "HC :");
                oled_show_string(5, 1, selected_item == 3 ? ">HMR:" : "HMR:");
                oled_show_string(6, 1, selected_item == 4 ? ">VR :" : "VR :");
                oled_show_string(7, 1, selected_item == 5 ? ">HR :" : "HR :");
                oled_show_string(8, 1, selected_item == 6 ? ">VL :" : "VL :");

                oled_show_num(2, 8, max_value[0], 4);
                oled_show_num(3, 8, max_value[1], 4);
                oled_show_num(4, 8, max_value[2], 4);
                oled_show_num(5, 8, max_value[3], 4);
                oled_show_num(6, 8, max_value[4], 4);
                oled_show_num(7, 8, max_value[5], 4);
                oled_show_num(8, 8, max_value[6], 4);
            }
            break;

        case PID_PARA:
            {
                oled_show_string(1, 8, "pidpara");
                
                // 显示PID参数名称，选中项前添加'>'标记
                oled_show_string(2, 1, selected_item == 0 ? ">S_Kp:" : "S_Kp:");
                oled_show_string(3, 1, selected_item == 1 ? ">S_Ki:" : "S_Ki:");
                oled_show_string(4, 1, selected_item == 2 ? ">S_Kd:" : "S_Kd:");
                oled_show_string(5, 1, selected_item == 3 ? ">T_Kp:" : "T_Kp:");
                oled_show_string(6, 1, selected_item == 4 ? ">T_Ki:" : "T_Ki:");
                oled_show_string(7, 1, selected_item == 5 ? ">T_Kd:" : "T_Kd:");

                oled_show_float(2, 8, SpeedPID.kp);
                oled_show_float(3, 8, SpeedPID.ki);
                oled_show_float(4, 8, SpeedPID.kd);
                oled_show_float(5, 8, TurnPID.kp);
                oled_show_float(6, 8, TurnPID.ki);
                oled_show_float(7, 8, TurnPID.kd);
            }
            break;

        case CHARGE:
            {
                oled_show_string(1, 1, "charge");
            }
            break;

        case RUNNING:
            {
                oled_show_string(1, 1, "running");
            }
            break;
        
        default:
            break;
    }
}