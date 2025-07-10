/*************	头文件	**************/
#include "headfile.h"
#include "at24c16.h"
#include "STC32G_ADC.h"
#include "STC32G_DMA.h"
#include "STC32G_Switch.h"

/*************	宏定义	**************/
#define ADC_HL  14  // 左侧横向电感-P0.6
#define ADC_VL  13  // 左侧纵向电感-P0.5
#define ADC_HML 9   // 左中横向电感-P0.1
#define ADC_HC  1   // 中间横向电感-P1.1
#define ADC_HMR 3   // 右中横向电感-P1.3
#define ADC_VR  4   // 右侧纵向电感-P1.4
#define ADC_HR  8   // 右侧横向电感-P0.0

#define	ADC_CH		7			/* 1~16, ADC转换通道数, 需同步修改转换通道 */
#define	ADC_DATA	6			/* 6~n, 每个通道ADC转换数据总数, 2*转换次数+4, 需同步修改转换次数 */


/*************	全局变量	**************/
u8 chn = 0;
u8 xdata DmaAdBuffer[ADC_CH][ADC_DATA];
uint8_t g_TxData[200] = {0};


/*************	函数声明	**************/
void PrintChAvg7(void);
void ADC_config(void);
void DMA_config(void);
void GPIO_config(void);



/*************	主函数	**************/
void main(void)
{
	/*************	本地变量声明	**************/
	int state = 5;
	uint16 sum_value = 0;    
	uint16 value[7] = {0};   //调试用数组

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
	pit_timer_ms(TIM_2, 5);

	pid_init(&SpeedPID, 1.0f, 2.0f, 3.0f, 5000.0f, 6000.0f); //初始化速度PID
	pid_init(&TurnPID, 1.0f, 2.0f, 3.0f, 0.0f, 6000.0f);  //初始化位置PID
	lowpass_init(&leftSpeedFilt, 0.556);   //初始化低通滤波器
	lowpass_init(&rightSpeedFilt, 0.556);
	kalman_init(&imu693_kf, 0.98, 0.02, imu693kf_Q, imu693kf_R, 0.0);
	
    /* 从EEPROM加载max_value及PID参数，覆盖默认值 */
   load_parameters_from_eeprom();
//	save_parameters_to_eeprom();  //保存max_value及PID参数到EEPROM（初始化）

	/*************	主循环	**************/
    while(1)
	{
//		 key_task();         // 处理按键任务
//		 display_task();     // OLED显示任务
		
		uart4_recv_task();  // 串口4接收任务
		
//		sprintf(g_TxData, "%f,%f\n",Gyro_Z,filtered_GyroZ);
//		uart_putstr(UART_4, g_TxData);

		/*************	ADC DMA采样完成	**************/
		if(DmaADCFlag)  //判断ADC DMA采样是否完成
		{
			DmaADCFlag = 0; //清除完成标志

			PrintChAvg7();

			DMA_ADC_TRIG();		//重新触发启动下一次转换
		}
		
#if 0
		// 通过串口输出七电感数据
		sprintf(g_TxData, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		 (uint16)normalized_data[SENSOR_HL], 
		 (uint16)normalized_data[SENSOR_VL], 
		 (uint16)normalized_data[SENSOR_HML], 
		 (uint16)normalized_data[SENSOR_HC],
		 (uint16)normalized_data[SENSOR_HMR], 
		 (uint16)normalized_data[SENSOR_VR], 
		 (uint16)normalized_data[SENSOR_HR], 
		  position,
		 (uint16)signal_strength_value,
		  track_type,
		  track_route,
		  track_route_status,
		  track_type_zj);
		 uart_putstr(UART_4, g_TxData);


		// 获取滤波后的ADC数据		
		mid_filter();      // 使用中位值滤波获取电感数据

		// 归一化电感数组·
		normalize_sensors();
		
		// 计算位置偏差
		position = calculate_position_improved();
		
		//检查电磁保护
		protection_flag = check_electromagnetic_protection();
#endif		
		
		/* 调试功能 */
#if 0
		//读取七电感ADC值（用于调试）
		value[0] = adc_once(ADC_HL,  ADC_10BIT);
		value[1] = adc_once(ADC_VL,  ADC_10BIT);
		value[2] = adc_once(ADC_HML, ADC_10BIT);
		value[3] = adc_once(ADC_HC,  ADC_10BIT); 
		value[4] = adc_once(ADC_HMR, ADC_10BIT);
		value[5] = adc_once(ADC_VR,  ADC_10BIT);
		value[6] = adc_once(ADC_HR,  ADC_10BIT);	

		// 计算所有电感值的总和
//		sum_value = (uint16)normalized_data[SENSOR_HL] + (uint16)normalized_data[SENSOR_VL] + 
//		            (uint16)normalized_data[SENSOR_HML] + (uint16)normalized_data[SENSOR_HC] + 
//		            (uint16)normalized_data[SENSOR_HMR] + (uint16)normalized_data[SENSOR_VR] + 
//		            (uint16)normalized_data[SENSOR_HR];

		 // 通过串口输出七电感原始数据
		  sprintf(g_TxData, "%d,%d,%d,%d,%d,%d,%d\n",
					value[0], 
					value[1], 
					value[2], 
					value[3], 
					value[4],
					value[5],
          value[6]);
		  uart_putstr(UART_4, g_TxData);

		  delay_ms(10);
#endif	
		
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
	DMA_ADC_InitStructure.DMA_Channel = 0x631A;			//ADC通道使能: P0.0, P0.1, P0.5, P0.6, P1.1, P1.3, P1.4
	DMA_ADC_InitStructure.DMA_Buffer = (u16)DmaAdBuffer;	//ADC转换数据存储地址
	DMA_ADC_InitStructure.DMA_Times = ADC_1_Times;	//每个通道转换次数, ADC_1_Times,ADC_2_Times,ADC_4_Times,ADC_8_Times,ADC_16_Times,ADC_32_Times,ADC_64_Times,ADC_128_Times,ADC_256_Times
	DMA_ADC_Inilize(&DMA_ADC_InitStructure);		//初始化
	NVIC_DMA_ADC_Init(ENABLE,Priority_0,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0~Priority_3; 总线优先级(低到高) Priority_0~Priority_3
	DMA_ADC_TRIG();		//触发启动转换
}

void PrintChAvg7(void)	
{
    // 一次性打印前 7 个通道的平均值 (假设 ADC_CH >= 7)
    u16 HC = ((u16)DmaAdBuffer[0][4] << 8) | DmaAdBuffer[0][5];//1
    u16 HMR = ((u16)DmaAdBuffer[1][4] << 8) | DmaAdBuffer[1][5];//3
    u16 VR = ((u16)DmaAdBuffer[2][4] << 8) | DmaAdBuffer[2][5];//4
    u16 HR = ((u16)DmaAdBuffer[3][4] << 8) | DmaAdBuffer[3][5];//8
    u16 HML = ((u16)DmaAdBuffer[4][4] << 8) | DmaAdBuffer[4][5];//9
    u16 VL = ((u16)DmaAdBuffer[5][4] << 8) | DmaAdBuffer[5][5];//13
    u16 HL = ((u16)DmaAdBuffer[6][4] << 8) | DmaAdBuffer[6][5];//14

    sprintf(g_TxData,"%u,%u,%u,%u,%u,%u,%u\r\n",
           HL, VL, HML, HC, HMR, VR, HR);
    uart_putstr(UART_4, g_TxData);
    delay_ms(20);
}
