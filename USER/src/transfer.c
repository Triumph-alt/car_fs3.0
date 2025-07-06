#include "transfer.h"
#include "OLED.h"
#include "electromagnetic_tracking.h"
#include "pid.h"

Key_t key[4] = {0, 0, 0, 0};
enum state car_state; 
uint8 selected_item = 0; // 当前选中的项目索引

void key_task(void)
{
	if (key[0].flag == 1)
	{
        if (car_state < RUNNING)
        {
            car_state++;
            oled_clear();
            selected_item = 0; // 切换状态时重置选中项
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