/*
 * TimDMA.c
 *
 *  Created on: Dec 9, 2019
 *      Author: fil
 */

#include "stm32g4xx_hal.h"
#include "main.h"

extern	TIM_HandleTypeDef htim1;
extern 	TIM_HandleTypeDef htim6;
static void fast_DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Clear the DMAMUX synchro overrun flag */
  hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

  if (hdma->DMAmuxRequestGen != 0U)
  {
    /* Clear the DMAMUX request generator overrun flag */
    hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
  }

  /* Clear all flags */
  hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1FU));

  /* Configure DMA Channel data length */
  hdma->Instance->CNDTR = DataLength;

  /* Memory to Peripheral */
    /* Configure DMA Channel destination address */
    hdma->Instance->CPAR = DstAddress;

    /* Configure DMA Channel source address */
    hdma->Instance->CMAR = SrcAddress;
}

HAL_StatusTypeDef fast_HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress,
                                   uint32_t DataLength)
{
	/* Disable the peripheral */
	__HAL_DMA_DISABLE(hdma);

	/* Configure the source, destination address and the data length & clear flags*/
	fast_DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);

	__HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
	__HAL_DMA_ENABLE_IT(hdma, (DMA_IT_TC | DMA_IT_TE));

	/* Enable the Peripheral */
	__HAL_DMA_ENABLE(hdma);
	return HAL_OK;
}

//TIM1_CHANNEL_1
void PWM_Start_DMA_Tim1(uint32_t *pData, uint16_t Length)
{
	/* Set the DMA error callback */
	htim1.hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError ;
	/* Enable the DMA channel */
	fast_HAL_DMA_Start_IT( htim1.hdma[TIM_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim1.Instance->PSC, Length);
	/* Enable the TIM Capture/Compare 1 DMA request */
	htim1.Instance->DIER |= TIM_DMA_CC1;
	/* Enable the Capture compare channel */
	TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	//HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); //starts PWM on CH1N pin
	htim1.Instance->BDTR|=TIM_BDTR_MOE;
	/* Enable the Peripheral */
	//htim1.Instance->CR1|=TIM_CR1_CEN;
}

void tim6_start(void)
{
  /* Enable the TIM Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  __HAL_TIM_ENABLE(&htim6);
}


