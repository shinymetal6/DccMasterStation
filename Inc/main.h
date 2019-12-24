/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENSE_Pin GPIO_PIN_0
#define SENSE_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_9
#define ENABLE_GPIO_Port GPIOA
#define DEBUG_FLAG_Pin GPIO_PIN_10
#define DEBUG_FLAG_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define SENSE_OUT_Pin GPIO_PIN_6
#define SENSE_OUT_GPIO_Port GPIOB
#define TIM16_CH1_LED_Pin GPIO_PIN_8
#define TIM16_CH1_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern	void init_bufs(void);
extern	void PWM_Start_DMA_Tim1(uint32_t *pData, uint16_t Length);
extern	void tim6_start(void);
extern	void uart_start(void);
extern	void uart_tx_buffer(uint8_t *pData);
extern	void usb_tx_buffer(uint8_t *pData);
extern	void packet_callback(void);
extern	void cutout_callback(void);
extern	void parser(uint32_t source_buf);
extern	void debug_flag_pulse(void);
extern	uint32_t encode_throttle(int cab,int speed,int direction);
extern	void insert_len3_packet(void);
extern	void insert_len4_packet(void);
extern	void insert_reset_packet(void);

extern	uint32_t USB_UART_RxCpltCallback(void);

#define		BIT_0		340
#define		BIT_1		170
#define		PREAMBLE_LEN	15
#define		BYTE_LEN		8
#define		CLOSER_LEN		2

#define		PACKET_LEN	256
typedef struct {
	uint16_t 	main_power;
} s_main_track;

typedef struct {
	uint16_t preamble[PREAMBLE_LEN];
	uint16_t byte1_start;
	uint16_t byte1[BYTE_LEN];
	uint16_t byte2_start;
	uint16_t byte2[BYTE_LEN];
	uint16_t errdet_start;
	uint16_t errcheck[BYTE_LEN];
	uint16_t closer[CLOSER_LEN];
} s_len3_packet;

typedef struct {
	uint16_t preamble[PREAMBLE_LEN];
	uint16_t byte1_start;
	uint16_t byte1[BYTE_LEN];
	uint16_t byte2_start;
	uint16_t byte2[BYTE_LEN];
	uint16_t byte3_start;
	uint16_t byte3[BYTE_LEN];
	uint16_t errdet_start;
	uint16_t errcheck[BYTE_LEN];
	uint16_t closer[CLOSER_LEN];
} s_len4_packet;

typedef struct {
	uint16_t preamble[PREAMBLE_LEN];
	uint16_t byte1_start;
	uint16_t byte1[BYTE_LEN];
	uint16_t byte2_start;
	uint16_t byte2[BYTE_LEN];
	uint16_t byte3_start;
	uint16_t byte3[BYTE_LEN];
	uint16_t byte4_start;
	uint16_t byte4[BYTE_LEN];
	uint16_t errdet_start;
	uint16_t errcheck[BYTE_LEN];
	uint16_t closer[CLOSER_LEN];
} s_len5_packet;

typedef struct {
	uint16_t preamble[PREAMBLE_LEN];
	uint16_t byte1_start;
	uint16_t byte1[BYTE_LEN];
	uint16_t byte2_start;
	uint16_t byte2[BYTE_LEN];
	uint16_t byte3_start;
	uint16_t byte3[BYTE_LEN];
	uint16_t byte4_start;
	uint16_t byte4[BYTE_LEN];
	uint16_t byte5_start;
	uint16_t byte5[BYTE_LEN];
	uint16_t errdet_start;
	uint16_t errcheck[BYTE_LEN];
	uint16_t closer[CLOSER_LEN];
} s_len6_packet;

/* UART */

#define	USBUART_BUFLEN	15
typedef struct {
	uint8_t byte_count;
	uint8_t packet[USBUART_BUFLEN-1];
} s_rxbuf;

#define	FROM_UART	0
#define	FROM_USB	1

#define	TIM16_BLINK_INACTIVE	16800
#define	TIM16_BLINK_ACTIVE		TIM16_BLINK_INACTIVE / 4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
