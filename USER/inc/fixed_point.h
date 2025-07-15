/// fixed_point.h  (任务4.1)
#ifndef __FIXED_POINT_H
#define __FIXED_POINT_H

#include "common.h"    // 提供标准整型定义

// 定点数放大倍数（可根据精度需求调整）
#define FX_SCALE 1000

// 后续任务(4.2)将补充定点数运算宏与内联函数

// === 基础转换宏 ===
// 将浮点常量转换为定点数（四舍五入）
#define FLOAT_TO_FIXED(x) ((int32_t)((x) * FX_SCALE + ((x) >= 0 ? 0.5f : -0.5f)))
// 将整数转换为定点数
#define INT_TO_FIXED(x)   ((int32_t)(x) * FX_SCALE)
// 将定点数转换为浮点（调试用，不建议在实时循环中频繁调用）
#define FIXED_TO_FLOAT(x) ((float)(x) / FX_SCALE)

// === 运算宏 ===
// 定点乘法: (a * b) / SCALE
#define FIXED_MUL(a, b)   ((int32_t)(((a) * (b)) / FX_SCALE))
// 定点除法: (a * SCALE) / b
#define FIXED_DIV(a, b)   ((int32_t)(((a) * FX_SCALE) / (b)))

// === 常用宏 ===
// 获取定点数的绝对值
#define FIXED_ABS(x)      ((x) >= 0 ? (x) : -(x))

#endif // __FIXED_POINT_H 


