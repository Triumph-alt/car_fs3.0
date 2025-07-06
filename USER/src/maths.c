#include "maths.h"

int myabs(int num)
{
	return (num > 0) ? num : -num;
}

float myfabs(float num)
{
	return (num > 0) ? num : -num;
}

///**
//  * @brief  矩阵乘法函数
//  * @param  A: 第一个矩阵，rowsA x colsA
//  * @param  rowsA: 第一个矩阵的行数
//  * @param  colsA: 第一个矩阵的列数
//  * @param  B: 第二个矩阵，rowsB x colsB
//  * @param  rowsB: 第二个矩阵的行数
//  * @param  colsB: 第二个矩阵的列数
//  * @param  C: 结果矩阵，rowsA x colsB
//  * @retval 无
//  */
//void Matrix_Mult(float* A, uint8_t rowsA, uint8_t colsA,
//                 float* B, uint8_t rowsB, uint8_t colsB,
//                 float* C) 
//{
//	uint8_t i = 0, j = 0, k = 0;
//	
//    if (colsA != rowsB) // 错误：矩阵维度不匹配
//	{
//        return;
//    }

//    for (i = 0; i < rowsA; i++)
//	{
//        for (j = 0; j < colsB; j++) 
//		{
//            float sum = 0.0f;

//            for (k = 0; k < colsA; k++) 
//			{
//                sum += A[i * colsA + k] * B[k * colsB + j];
//            }

//            C[i * colsB + j] = sum;
//        }
//    }
//}

///**
//  * @brief  矩阵缩放函数
//  * @param  A: 输入矩阵，rowsA x colsA
//  * @param  rowsA: 矩阵的行数
//  * @param  colsA: 矩阵的列数
//  * @param  scalar: 缩放因子
//  * @param  C: 输出矩阵，rowsA x colsA
//  * @retval 无
//  */
//void Matrix_Scale(float* A, uint8_t rowsA, uint8_t colsA, float scalar, float* C) 
//{
//	uint16_t i = 0;
//	
//    for (i = 0; i < rowsA * colsA; i++) 
//	{
//        C[i] = A[i] * scalar;
//    }
//}

//void Matrix_Negate(float* A, uint8_t rowsA, uint8_t colsA, float* C)
//{
//	uint16_t i = 0;
//	
//    for (i = 0; i < rowsA * colsA; i++) 
//	{
//        C[i] = -A[i];
//    }
//}