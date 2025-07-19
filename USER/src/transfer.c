#include "transfer.h"
#include "OLED.h"
#include "electromagnetic_tracking.h"
#include "pid.h"
#include "at24c16.h"

extern float result[7]; // 声明外部变量
// 声明环岛相关的外部变量
extern volatile uint8_t intoisland_pos;            // 入环岛的偏差
extern volatile uint16_t intoisland_str_dist;      // 入环岛直走距离
extern volatile uint16_t intoisland_all_dist;      // 入环岛总距离
extern volatile uint8_t outisland_pos;             // 出环岛的偏差
extern volatile uint16_t outisland_turn_dist;      // 出环岛拐弯距离
extern volatile uint16_t outisland_all_dist;       // 出环岛总距离

extern uint32 power_voltage; // 电池电压（毫伏）

Key_t key[4] = {0, 0, 0, 0, 0};
enum state car_state = CHARGE, prev_state; 
uint8 selected_item = 0; // 当前选中的项目索引

uint8_t startKeyFlag = 0, uartSendFlag = 1;

void key_task(void)
{
	if (key[0].short_flag == 1)
	{
#if NORMALRUN
		prev_state = car_state;   // 记录切换前状态
		
		car_state++;
		if (car_state == STRAIGHT)
		{
		   car_state = CHARGE;
		}
		
		oled_clear();
		selected_item = 0; // 切换状态时重置选中项

		/* 若从 ISLAND_PARA 切换到 CHARGE，保存参数 */
//		if(prev_state == ISLAND_PARA && car_state == CHARGE)
//		{
		save_parameters_to_eeprom();
//		}
#else
		if (startKeyFlag == 1)
		{
			set_motor_pwm(0, 0);

			pid_clean(&SpeedPID);
			pid_clean(&TurnPID);
		
			uartSendFlag = startKeyFlag = 0;
		}
		else
		{
			delay_ms(2000);
			uartSendFlag = startKeyFlag = 1;
		}
#endif

		key[0].short_flag = 0;
	}

	/* 按键2短按 */
	if (key[1].short_flag == 1)
	{
        // 按键2作为选择按键
        switch (car_state)
        {
            case ELECT_PARA:
                // 在电感参数界面，最多7个选项(max_value数组)
                selected_item = (selected_item + 1) % 7;
                break;
                
            case PID_PARA:
                // 在PID参数界面，最多6个选项(SpeedPID和TurnPID的kp,ki,kd)
                selected_item = (selected_item + 1) % 8;
                break;
                
            case ISLAND_PARA:
                // 在环岛参数界面，最多6个选项
                selected_item = (selected_item + 1) % 6;
                break;
                
            default:
                break;
        }
		
		key[1].short_flag = 0;
	}
	
	/* 按键3短按和长按 */
	if (key[2].long_flag == 1 || key[2].short_flag == 1)
	{
        // 按键3作为增加值的按键
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
                if (selected_item == 0) speed_kp += 1.0f;
                else if (selected_item == 1) speed_ki += 0.01f;
                else if (selected_item == 2) turn_kp += 1.0f;
                else if (selected_item == 3) turn_kd += 0.1f;
                else if (selected_item == 4) angle_kp += 1.0f;
                else if (selected_item == 5) angle_kd += 0.1f;
                else if (selected_item == 6) SPEED_STRAIGHT += 1;
                else if (selected_item == 7) SPEED_ISLAND += 1;
                break;
                
            case ISLAND_PARA:
                // 增加环岛参数
                if (selected_item == 0) intoisland_pos += 1;
                else if (selected_item == 1) intoisland_str_dist += 100;
                else if (selected_item == 2) intoisland_all_dist += 100;
                else if (selected_item == 3) outisland_pos += 1;
                else if (selected_item == 4) outisland_turn_dist += 100;
                else if (selected_item == 5) outisland_all_dist += 100;
                break;
                
            default:
                break;
        }
		
		if (key[2].long_flag == 1)
		{
			key[2].long_flag = 0;
		}
		else if (key[2].short_flag == 1)
		{
			key[2].short_flag = 0;
		}
	}

	/* 按键4短按和长按 */
	if (key[3].short_flag == 1 || key[3].long_flag == 1)
	{
        // 按键4作为减少值的按键
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
                if (selected_item == 0 && speed_kp >= 1.0f) speed_kp -= 1.0f;
                else if (selected_item == 1 && speed_ki >= 0.1f) speed_ki -= 0.01f;
                else if (selected_item == 2 && turn_kp >= 1.0f) turn_kp -= 1.0f;
                else if (selected_item == 3 && turn_kd >= 0.1f) turn_kd -= 0.1f;
                else if (selected_item == 4 && angle_kp >= 1.0f) angle_kp -= 1.0f;
                else if (selected_item == 5 && angle_kd >= 0.1f) angle_kd -= 0.1f;
                else if (selected_item == 6 && SPEED_STRAIGHT > 0) SPEED_STRAIGHT -= 1;
                else if (selected_item == 7 && SPEED_ISLAND > 0) SPEED_ISLAND -= 1;
                break;
                
            case ISLAND_PARA:
                // 减少环岛参数
                if (selected_item == 0 && intoisland_pos > 1) intoisland_pos -= 1;
                else if (selected_item == 1 && intoisland_str_dist >= 100) intoisland_str_dist -= 100;
                else if (selected_item == 2 && intoisland_all_dist >= 100) intoisland_all_dist -= 100;
                else if (selected_item == 3 && outisland_pos > 1) outisland_pos -= 1;
                else if (selected_item == 4 && outisland_turn_dist >= 100) outisland_turn_dist -= 100;
                else if (selected_item == 5 && outisland_all_dist >= 100) outisland_all_dist -= 100;
                break;
                
            default:
                break;
        }
		
		if (key[3].long_flag == 1)
		{
			key[3].long_flag = 0;
		}
		else if (key[3].short_flag == 1)
		{
			key[3].short_flag = 0;
		}
	}
}

/* 8行，21列*/
void display_task(void)
{
    switch (car_state)
    {
        case ELECT_PARA:
            {
                oled_show_string(1, 8, "max_v");
                oled_show_string(1, 14, "real_v");
                
                // 显示电感名称，选中项前添加'>'标记
                oled_show_string(2, 1, selected_item == 0 ? ">HL:" : "HL: ");
                oled_show_string(3, 1, selected_item == 1 ? ">VL:" : "VL: ");
                oled_show_string(4, 1, selected_item == 2 ? ">HML:" : "HML: ");
                oled_show_string(5, 1, selected_item == 3 ? ">HC:" : "HC: ");
                oled_show_string(6, 1, selected_item == 4 ? ">HMR:" : "HMR: ");
                oled_show_string(7, 1, selected_item == 5 ? ">VR:" : "VR: ");
                oled_show_string(8, 1, selected_item == 6 ? ">HR:" : "HR: ");

                oled_show_num(2, 8, max_value[0], 4);
                oled_show_num(3, 8, max_value[1], 4);
                oled_show_num(4, 8, max_value[2], 4);
                oled_show_num(5, 8, max_value[3], 4);
                oled_show_num(6, 8, max_value[4], 4);
                oled_show_num(7, 8, max_value[5], 4);
                oled_show_num(8, 8, max_value[6], 4);
				
				// 根据标签显示对应的实时电感值
                oled_show_num(2, 14, (uint16)result[0], 4); // HL
                oled_show_num(3, 14, (uint16)result[1], 4); // VL
                oled_show_num(4, 14, (uint16)result[2], 4); // HML
                oled_show_num(5, 14, (uint16)result[3], 4); // HC
                oled_show_num(6, 14, (uint16)result[4], 4); // HMR
                oled_show_num(7, 14, (uint16)result[5], 4); // VR
                oled_show_num(8, 14, (uint16)result[6], 4); // HR
            }
            break;

        case PID_PARA:
            {
                oled_show_string(1, 8, "pidpara");
                
                // 显示PID参数名称，选中项前添加'>'标记
                oled_show_string(2, 1, selected_item == 0 ? ">S_Kp:" : "S_Kp: ");
                oled_show_string(3, 1, selected_item == 1 ? ">S_Ki:" : "S_Ki: ");
                oled_show_string(4, 1, selected_item == 2 ? ">T_Kp:" : "T_Kp: ");
                oled_show_string(5, 1, selected_item == 3 ? ">T_Kd:" : "T_Kd: ");
                oled_show_string(6, 1, selected_item == 4 ? ">A_Kp:" : "A_Kp: ");
                oled_show_string(7, 1, selected_item == 5 ? ">A_Kd:" : "A_Kd: ");
                oled_show_string(8, 1, selected_item == 6 ? ">SP:" : "SP: ");
                oled_show_string(8, 8, selected_item == 7 ? ">IP:" : "IP: ");


                oled_show_float(2, 8, speed_kp);
                oled_show_float(3, 8, speed_ki);
                oled_show_float(4, 8, turn_kp);
                oled_show_float(5, 8, turn_kd);
                oled_show_float(6, 8, angle_kp);
                oled_show_float(7, 8, angle_kd);
                oled_show_num(8, 5, SPEED_STRAIGHT, 2);
                oled_show_num(8, 13, SPEED_ISLAND, 2);
            }
            break;

        case ISLAND_PARA:
            {
                // 显示入环岛参数
                oled_show_string(1, 10, "In Island");
                oled_show_string(2, 1, selected_item == 0 ? ">ipos:" : "ipos: ");
                oled_show_num(2, 10, intoisland_pos, 3);
                oled_show_string(3, 1, selected_item == 1 ? ">istr_d:" : "istr_d: ");
                oled_show_num(3, 10, intoisland_str_dist, 5);
                oled_show_string(4, 1, selected_item == 2 ? ">iall_d:" : "iall_d: ");
                oled_show_num(4, 10, intoisland_all_dist, 5);
                
                // 显示出环岛参数
                oled_show_string(5, 10, "Out Island");
                oled_show_string(6, 1, selected_item == 3 ? ">opos:" : "opos: ");
                oled_show_num(6, 10, outisland_pos, 3);
                oled_show_string(7, 1, selected_item == 4 ? ">oturn_d:" : "oturn_d: ");
                oled_show_num(7, 10, outisland_turn_dist, 5);
                oled_show_string(8, 1, selected_item == 5 ? ">oall_d:" : "oall_d: ");
                oled_show_num(8, 10, outisland_all_dist, 5);
            }
            break;
        case CHARGE:
            {
                oled_show_string(1, 1, "charge");
                oled_show_string(2, 1, "power:");
                oled_show_num(2, 8, power_voltage, 4);

                if (power_voltage > 1300)
                {
                    selected_item = 0;
                    oled_clear();
                    prev_state = CHARGE;
                    car_state = STRAIGHT;
                }
            }
            break;
        
        default:
            break;
    }
}