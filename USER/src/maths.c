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
//  * @brief  ����˷�����
//  * @param  A: ��һ������rowsA x colsA
//  * @param  rowsA: ��һ�����������
//  * @param  colsA: ��һ�����������
//  * @param  B: �ڶ�������rowsB x colsB
//  * @param  rowsB: �ڶ������������
//  * @param  colsB: �ڶ������������
//  * @param  C: �������rowsA x colsB
//  * @retval ��
//  */
//void Matrix_Mult(float* A, uint8_t rowsA, uint8_t colsA,
//                 float* B, uint8_t rowsB, uint8_t colsB,
//                 float* C) 
//{
//	uint8_t i = 0, j = 0, k = 0;
//	
//    if (colsA != rowsB) // ���󣺾���ά�Ȳ�ƥ��
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
//  * @brief  �������ź���
//  * @param  A: �������rowsA x colsA
//  * @param  rowsA: ���������
//  * @param  colsA: ���������
//  * @param  scalar: ��������
//  * @param  C: �������rowsA x colsA
//  * @retval ��
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