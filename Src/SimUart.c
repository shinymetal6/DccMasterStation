/*
 * SimUart.c
 *
 *  Created on: Dec 9, 2019
 *      Author: fil
 */

#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern		UART_HandleTypeDef huart2;

extern 		uint8_t uart2_rx_buffer_ready;
uint8_t 	uart_header=0;
uint16_t 	uart_busy_flag=0;
uint16_t 	uart_rx_index = 0;
extern 		s_rxbuf uart_rxbuf;

uint8_t 	usb_header=0;
uint16_t 	usb_busy_flag=0;
uint16_t 	usb_rx_index=0 , usb_received_bytes = 0;
extern 		s_rxbuf usb_rxbuf;
extern		uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void uart_tx_buffer(uint8_t *pData)
{
	uart_busy_flag = 1;
	HAL_UART_Transmit_DMA(&huart2, pData, strlen((char *)pData));
}
void usb_tx_buffer(uint8_t *pData)
{
	usb_busy_flag = 1;
	CDC_Transmit_FS(pData,strlen((char *)pData));
}
void uart_start(void)
{
	HAL_UART_Receive_IT(&huart2, &uart_rxbuf.packet[0], 1);
	uart_tx_buffer((uint8_t *)("Hyera DCC Command Station\n\r"));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ( uart_header == 0 )
	{
		if (uart_rxbuf.packet[0]== '<')
			uart_header = uart_rx_index = 1;
		HAL_UART_Receive_IT(&huart2, &uart_rxbuf.packet[uart_rx_index], 1);
	}
	else
	{
		if( uart_rxbuf.packet[uart_rx_index] == '>')
		{
			uart2_rx_buffer_ready=1;
			uart_rxbuf.byte_count = uart_rx_index;
			uart_header = uart_rx_index = 0;
			HAL_UART_Receive_IT(&huart2, &uart_rxbuf.packet[0], 1);
		}
		else
		{
			uart_rx_index++;
			if ( uart_rx_index >= USBUART_BUFLEN)
				uart_header = uart_rx_index = 0;
			HAL_UART_Receive_IT(&huart2, &uart_rxbuf.packet[uart_rx_index], 1);
		}
	}
}

uint32_t USB_UART_RxCpltCallback(void)
{
	if ( usb_header == 0 )
	{
		if (usb_rxbuf.packet[0]== '<')
			usb_header = usb_rx_index = 1;
	}
	else
	{
		if( usb_rxbuf.packet[usb_rx_index] == '>')
		{
			uart2_rx_buffer_ready=1;
			usb_rxbuf.byte_count = usb_rx_index;
			usb_header = usb_rx_index = 0;
			return 1; /* Return 1 when packet finished */
		}
		else
		{
			usb_rx_index++;
			if ( usb_rx_index >= USBUART_BUFLEN)
				usb_header = usb_rx_index = 0;
		}
	}
	return 0;
}

