#include "transfer.h"
#include "OLED.h"

Key_t key[4] = {0, 0, 0};
enum state car_state; 

void key_task(void)
{
	if (key[0].flag == 1)
	{
        if (car_state < RUNNING)
        {
            car_state++;
            oled_clear();
        }
		
		key[0].flag = 0;
	}

	if (key[1].flag == 1)
	{

		
		key[1].flag = 0;
	}

	if (key[2].flag == 1)
	{

		
		key[2].flag = 0;
	}

	if (key[3].flag == 1)
	{

		
		key[3].flag = 0;
	}
}

void display_task(void)
{
    switch (car_state)
    {
        case ELECT_PARA:
            {
                oled_show_string(1, 1, "electpara");
            }
            break;

        case PID_PARA:
            {
                oled_show_string(1, 1, "pidpara");
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