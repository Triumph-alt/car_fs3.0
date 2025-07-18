#ifndef __ELECTROMAGNETIC_TRACKING_H__
#define __ELECTROMAGNETIC_TRACKING_H__

#include "headfile.h"

// 电感通道定义
// #define ADC_HL  ADC_P06  // 左侧横向电感
// #define ADC_VL  ADC_P05  // 左侧纵向电感
// #define ADC_HML ADC_P01  // 左中横向电感
// #define ADC_HC  ADC_P11  // 中间横向电感
// #define ADC_HMR ADC_P13  // 右中横向电感
// #define ADC_VR  ADC_P14  // 右侧纵向电感
// #define ADC_HR  ADC_P00  // 右侧横向电感

/*************	宏定义	**************/
#define ADC_HL  14  // 左侧横向电感-P0.6
#define ADC_VL  13  // 左侧纵向电感-P0.5
#define ADC_HML 9   // 左中横向电感-P0.1
#define ADC_HC  1   // 中间横向电感-P1.1
#define ADC_HMR 3   // 右中横向电感-P1.3
#define ADC_VR  4   // 右侧纵向电感-P1.4
#define ADC_HR  8   // 右侧横向电感-P0.0

#define ADC_POWR 5

#define	ADC_CH		8				/* 1~16, ADC转换通道数, 需同步修改转换通道 */
#define ADC_TIMES   8 					/* 1~16, ADC转换次数, 需同步修改转换次数 */
#define	ADC_DATA	2*ADC_TIMES+4	/* 12, 每个通道ADC转换数据总数, 2*转换次数+4 (n = 4), 需同步修改转换次数 */


// 电感数组定义
#define SENSOR_COUNT 7   // 电感个数
#define HISTORY_COUNT 8  // 滤波次数，当前只存储最新值

// 赛道类型索引，与track_type对应
#define WEIGHT_STRAIGHT    0  // 直道
#define WEIGHT_RIGHT_ANGLE 1  // 直角弯道
#define WEIGHT_CROSS       2  // 十字圆环
#define WEIGHT_ROUNDABOUT  3  // 环岛

// 定义电感权重结构体
typedef struct {
    float weight_outer;    // 外侧电感权重(HL和HR)
    float weight_middle;   // 中间电感权重(HML和HMR)
    float weight_center;   // 中心电感权重(HC)
    float weight_vertical; // 纵向电感权重(VL和VR)
    float filter_param;    // 滤波系数，可调
    int16 max_change_rate; // 允许的最大变化率
    char *name;            // 赛道类型名称，便于调试
} TrackWeights;


// 电感类型枚举
typedef enum {
    SENSOR_HL  = 0,  // 左侧横向电感
    SENSOR_VL  = 1,  // 左侧纵向电感-8
    SENSOR_HML = 2,  // 左中横向电感
    SENSOR_HC  = 3,  // 中间横向电感
    SENSOR_HMR = 4,  // 右中横向电感
    SENSOR_VR  = 5,  // 右侧纵向电感
    SENSOR_HR  = 6   // 右侧横向电感
} sensor_type_e;

// 函数声明

void average_filter(void);                     // 递推均值滤波函数
void mid_filter(void);                         // 中位值滤波函数
void update_min_max_values(void);              // 更新每个电感的最大最小值
void normalize_sensors(void);                  // 归一化电感数据
int16 calculate_position_improved(void);      // 改进版计算位置
uint8 check_electromagnetic_protection(void);  // 电磁保护逻辑函数

// 外部变量声明
extern uint16 adc_fliter_data[SENSOR_COUNT][HISTORY_COUNT]; // 滤波后的值
extern float result[SENSOR_COUNT];                // 电感结果数据
extern float normalized_data[SENSOR_COUNT];       // 归一化后的电感数据，数组形式
extern int16 position;                         // 位置偏差
extern uint8 protection_flag;                  // 保护标志
extern float signal_strength_value;            // 信号强度指标
extern uint16 min_value[SENSOR_COUNT];
extern uint16 max_value[SENSOR_COUNT];

//电磁位置计算变量
extern float filter_param;      // 滤波系数，可调
extern uint8 track_type;        // 赛道类型：0-普通，1-十字，2-环岛，3-直角弯道
extern uint8 track_type_last;   // 赛道类型：0-普通，1-直角弯道，2-十字圆环，3-环岛

extern uint8 track_route;      // 赛道路径：1-左转，2-右转
extern uint8 track_route_status;
extern uint8 track_type_zj;

//计时判定变量
extern uint8 track_ten_flag; //十字圆环：0表示到计时0.5s再开始判断，1-可以开始判断
extern uint8 ten_ch_flag; //十字圆环：1表示0.5s后track_ten_flag=1
extern uint8 track_island_flag; //环岛：0表示到计时5s再开始判断，1-可以开始判断
extern uint8 island_ch_flag; //环岛：1表示5s后track_outisland_flag=1

// 电磁保护逻辑变量
extern uint8 protection_flag;



#endif 

