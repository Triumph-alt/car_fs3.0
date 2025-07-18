/*---------------------------------------------------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/*---------------------------------------------------------------------*/

#include "STC32G_DMA.h"

bit DmaADCFlag = 0;

//========================================================================
// 函数: DMA_ADC_ISR_Handler
// 描述: DMA ADC 中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2022-03-23
//========================================================================
void DMA_ADC_ISR_Handler (void) interrupt DMA_ADC_VECTOR
{
	// TODO: 在此处添加用户代码
	if(DMA_ADC_STA & 0x01)	//AD转换完成
	{
		DMA_ADC_STA &= ~0x01;	//清标志位
		DmaADCFlag = 1;
	}
}

//========================================================================
// 函数: DMA_ISR_Handler
// 描述: DMA中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2021-05-25
//========================================================================
void DMA_ISR_Handler (void) interrupt 13
{
	// TODO: 在此处添加用户代码
	
	//----------- DMA ADC --------------
	if(DMA_ADC_STA & 0x01)	//AD转换完成
	{
		DMA_ADC_STA &= ~0x01;	//清标志位
		DmaADCFlag = 1;
	}
}
