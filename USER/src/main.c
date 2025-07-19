/*************	头文件	**************/
#include "headfile.h"
#include "at24c16.h"
#include "STC32G_ADC.h"
#include "STC32G_DMA.h"
#include "STC32G_Switch.h"


/*************	全局变量	**************/
extern u8 xdata DmaAdBuffer[ADC_CH][ADC_DATA];
extern float result[SENSOR_COUNT];       //滤波后的电感值
extern uint32 power_voltage;

/*************	函数声明	**************/
void PrintChAvg7(void);
void ADC_config(void);
void DMA_config(void);
void GPIO_config(void);
void PrintFiltered7(void);               // 打印滤波后七电感数据
void Printtest(void);            
void PrintNormalized17(void);
void PrintDebugData(void);

/*************	主函数	**************/
void main(void)
{
	/*************	本地变量声明	**************/

	/*************	系统初始化	**************/
	board_init();			 // 初始化寄存器
	GPIO_config();			 //初始化外设
	ADC_config();
	DMA_config();
	iic_init(IIC_2, IIC2_SCL_P25, IIC2_SDA_P24, 0);
	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_4);
	motor_init();
	encoder_init();
	imu963ra_init();
	oled_init();

	pit_timer_ms(TIM_1, 10);
	pit_timer_ms(TIM_2, 1);

	pid_init(&SpeedPID, speed_kp, speed_ki, 0.0f, 8000.0f, 9000.0f);      //初始化速度PID
	pid_init(&TurnPID, turn_kp, 0.0f, turn_kd, 0.0f, 9000.0f);        //初始化位置PID
	lowpass_init(&leftSpeedFilt, 0.556);                          //初始化低通滤波器
	lowpass_init(&rightSpeedFilt, 0.556);
	kalman_init(&imu693_kf, 0.98, 0.02, imu693kf_Q, imu693kf_R, 0.0);
	
    /* 从EEPROM加载max_value及PID参数，覆盖默认值 */
#if NORMALRUN
	load_parameters_from_eeprom();
#endif
//	save_parameters_to_eeprom();  //保存max_value及PID参数到EEPROM（初始化）

	/*************	主循环	**************/
    while(1)
	{
		uart4_recv_task();  // 串口4接收任务
		key_task();         // 处理按键任务
#if NORMALRUN
		display_task();     // OLED显示任务
#endif
				
		/*************	 定时操作	**************/
		if (flag == 1)
		{
//			if (g_speedpoint == 60)
//			{
//				g_speedpoint = 150;
//			}
//			else if (g_speedpoint == 150)
//			{
//				g_speedpoint = 60;
//			}
			
			flag = 0;
		}

		/*************	ADC DMA采样完成	**************/
		if(DmaADCFlag)  //判断ADC DMA采样是否完成
		{
			// 使用average_filter读取DMA数据并完成递推均值滤波
			average_filter();
			// 重新触发DMA进行下一次转换
			DMA_ADC_TRIG();
		}
		
#if NORMALRUN
		/*************	充完电直走一段进赛道	**************/
		if (car_state == STRAIGHT)
		{
			set_motor_pwm(1500, 1500);
			
			if(signal_strength_value > 30.0f)
			{
				prev_state = STRAIGHT;
                car_state = RUNNING;
				
				startKeyFlag = 1; // 充电完成
			}
			
		}
#endif
		
		//归一化电感数组
		normalize_sensors();
	
		// 计算位置偏差
		position = calculate_position_improved();
		
		// 检查电磁保护
		if (!protection_flag && startKeyFlag == 1)
			protection_flag = check_electromagnetic_protection();
		
		// 打印数据
//		PrintNormalized17(); //原始数据和归一化
		PrintDebugData();	 //调试数据
//		Printtest(); 		 //电感元素判别
    }
}

/*************	GPIO 配置	**************/
void GPIO_config(void)
{
	gpio_mode(P0_0, GPI_IMPEDANCE);
	gpio_mode(P0_1, GPI_IMPEDANCE);
	gpio_mode(P0_5, GPI_IMPEDANCE);
	gpio_mode(P0_6, GPI_IMPEDANCE);
	gpio_mode(P1_1, GPI_IMPEDANCE);
	gpio_mode(P1_3, GPI_IMPEDANCE);
	gpio_mode(P1_4, GPI_IMPEDANCE);
	gpio_mode(P1_5, GPI_IMPEDANCE);
}


/*************	ADC 配置	**************/
void ADC_config(void)
{
	ADC_InitTypeDef		ADC_InitStructure;		//结构定义

	ADC_InitStructure.ADC_SMPduty   = 31;		//ADC 模拟信号采样时间控制, 0~31（注意： SMPDUTY 一定不能设置小于 10）
	ADC_InitStructure.ADC_CsSetup   = 0;		//ADC 通道选择时间控制 0(默认),1
	ADC_InitStructure.ADC_CsHold    = 1;		//ADC 通道选择保持时间控制 0,1(默认),2,3
	ADC_InitStructure.ADC_Speed     = ADC_SPEED_2X16T;		//设置 ADC 工作时钟频率	ADC_SPEED_2X1T~ADC_SPEED_2X16T
	ADC_InitStructure.ADC_AdjResult = ADC_RIGHT_JUSTIFIED;	//ADC结果调整,	ADC_LEFT_JUSTIFIED,ADC_RIGHT_JUSTIFIED
	ADC_Inilize(&ADC_InitStructure);		//初始化
	ADC_PowerControl(ENABLE);						//ADC电源开关, ENABLE或DISABLE
	NVIC_ADC_Init(DISABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}

/*************	DMA 配置	**************/
void DMA_config(void)
{
	DMA_ADC_InitTypeDef		DMA_ADC_InitStructure;		//结构定义

	DMA_ADC_InitStructure.DMA_Enable = ENABLE;			//DMA使能  	ENABLE,DISABLE
	// DMA_ADC_InitStructure.DMA_Channel = 0xffff;         //ADC通道使能寄存器, 1:使能, bit15~bit0 对应 ADC15~ADC0
	DMA_ADC_InitStructure.DMA_Channel = 0x633A;			//ADC通道使能: P0.0, P0.1, P0.5, P0.6, P1.1, P1.3, P1.4
	DMA_ADC_InitStructure.DMA_Buffer = (u16)DmaAdBuffer;	//ADC转换数据存储地址
	DMA_ADC_InitStructure.DMA_Times = ADC_8_Times;	//每个通道转换次数, ADC_1_Times,ADC_2_Times,ADC_4_Times,ADC_8_Times,ADC_16_Times,ADC_32_Times,ADC_64_Times,ADC_128_Times,ADC_256_Times
	DMA_ADC_Inilize(&DMA_ADC_InitStructure);		//初始化
	NVIC_DMA_ADC_Init(ENABLE,Priority_0,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3
	DMA_ADC_TRIG();		//触发启动转换
}


/*************	打印电感原始数据	**************/
void PrintChAvg7(void)	
{
    // 一次性打印前 7 个通道的平均值 (假设 ADC_CH >= 7)
    u16 HC  = ((u16)DmaAdBuffer[0][2*ADC_TIMES+2] << 8) | DmaAdBuffer[0][2*ADC_TIMES+3]; //1
    u16 HMR = ((u16)DmaAdBuffer[1][2*ADC_TIMES+2] << 8) | DmaAdBuffer[1][2*ADC_TIMES+3]; //3
    u16 VR  = ((u16)DmaAdBuffer[2][2*ADC_TIMES+2] << 8) | DmaAdBuffer[2][2*ADC_TIMES+3]; //4
    u16 HR  = ((u16)DmaAdBuffer[3][2*ADC_TIMES+2] << 8) | DmaAdBuffer[3][2*ADC_TIMES+3]; //8
    u16 HML = ((u16)DmaAdBuffer[4][2*ADC_TIMES+2] << 8) | DmaAdBuffer[4][2*ADC_TIMES+3]; //9
    u16 VL  = ((u16)DmaAdBuffer[5][2*ADC_TIMES+2] << 8) | DmaAdBuffer[5][2*ADC_TIMES+3]; //13
    u16 HL  = ((u16)DmaAdBuffer[6][2*ADC_TIMES+2] << 8) | DmaAdBuffer[6][2*ADC_TIMES+3]; //14

    sprintf(g_txbuffer,"%u,%u,%u,%u,%u,%u,%u\r\n",
           HL, VL, HML, HC, HMR, VR, HR);
    uart_putstr(UART_4, g_txbuffer);
}

/*************	打印滤波后电感数据	**************/
void PrintFiltered7(void)
{
    // 将 float 转为无符号整数打印，便于串口调试
    sprintf(g_txbuffer, "%u,%u,%u,%u,%u,%u,%u\r\n",
            (uint16)result[SENSOR_HL],
            (uint16)result[SENSOR_VL],
            (uint16)result[SENSOR_HML],
            (uint16)result[SENSOR_HC],
            (uint16)result[SENSOR_HMR],
            (uint16)result[SENSOR_VR],
            (uint16)result[SENSOR_HR]);
    uart_putstr(UART_4, g_txbuffer);
}

/*************	打印电感元素判别数据	**************/
void Printtest(void)
{
    // 将归一化后的float数据打印，保留两位小数
    sprintf(g_txbuffer, "%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u\r\n",
            (uint16)normalized_data[SENSOR_HL],
            (uint16)normalized_data[SENSOR_VL],
            (uint16)normalized_data[SENSOR_HML],
            (uint16)normalized_data[SENSOR_HC],
            (uint16)normalized_data[SENSOR_HMR],
            (uint16)normalized_data[SENSOR_VR],
            (uint16)normalized_data[SENSOR_HR],
			(uint16)signal_strength_value,
	        positionReal,
			track_type,
//			track_type_zj,
			track_route,
			track_route_status
//			g_intencoderALL
			);
    uart_putstr(UART_4, g_txbuffer);
}

/*************	打印原始和归一化数据	**************/
void PrintNormalized17(void)
{
    // 将归一化后的float数据打印，保留两位小数
    sprintf(g_txbuffer, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d\r\n",
            (uint16)result[SENSOR_HL],
            (uint16)result[SENSOR_VL],
            (uint16)result[SENSOR_HML],
            (uint16)result[SENSOR_HC],
            (uint16)result[SENSOR_HMR],
            (uint16)result[SENSOR_VR],
            (uint16)result[SENSOR_HR],
            (uint16)normalized_data[SENSOR_HL],
            (uint16)normalized_data[SENSOR_VL],
            (uint16)normalized_data[SENSOR_HML],
            (uint16)normalized_data[SENSOR_HC],
            (uint16)normalized_data[SENSOR_HMR],
            (uint16)normalized_data[SENSOR_VR],
            (uint16)normalized_data[SENSOR_HR],
            positionReal);
    uart_putstr(UART_4, g_txbuffer);
}

/*************	打印调试数据	**************/
void PrintDebugData(void)
{
    if (uartSendFlag == 1)
	{
		sprintf(g_txbuffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%d,%d\n", 
				g_speedpoint, 
				g_encoder_average, 
				EncoderL.encoder_final,
				EncoderR.encoder_final,
				(int)g_DutyLeft,
				(int)g_DutyRight,
				positionReal,
				(int)speed_pid,
				(int)turn_pid,
				(uint16)power_voltage,
				track_type,
				track_route,
				track_route_status,
				(int)SpeedPID.p_out,
				(int)SpeedPID.i_out
				);
		uart_putstr(UART_4, g_txbuffer);
				
		if (position >= 0)
			P52 = 1;
		else
			P52 = 0;
	}
}

