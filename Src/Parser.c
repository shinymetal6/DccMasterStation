/*
 * Parser.c
 *
 *  Created on: Dec 9, 2019
 *      Author: fil
 */


#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern  s_rxbuf uart_rxbuf,usb_rxbuf;
extern	void uart_tx_buffer(uint8_t *pData);
extern	s_main_track 	main_track;


extern	s_len3_packet	reset_packet;

char outbuf[64];

/* <f 1234 5678 9012> */
void one_byte_commands(char cmd)
{
	switch ( cmd)
	{
	case '0' 	: 	sprintf(outbuf,"P0 Main Off\n\r");
					main_track.main_power = 0;
					break;
	case '1' 	: 	sprintf(outbuf,"P1 Main On\n\r");
					main_track.main_power = 1;
					tim6_start();
					packet_callback();
					break;
	case '8' 	: 	sprintf(outbuf,"All Off\n\r");
					main_track.main_power = 0;
					break;
	case '9' 	: 	sprintf(outbuf,"All On\n\r");
					main_track.main_power = 1;
					tim6_start();
					packet_callback();
					break;
	case 'R' 	: 	sprintf(outbuf,"Sent RESET\n\r");
					insert_reset_packet();
					break;
	case 'S' 	: 	sprintf(outbuf,"Status\n\r");
					break;
	default:	sprintf(outbuf,"Command error\n\r");
	}
}

void three_bytes_commands(char cmd,int p0,int p1)
{
	switch ( cmd)
	{
	case 'f' 	: 	sprintf(outbuf,"f %d %d\n\r",p0,p1);
					break;
	case 'T' 	: 	sprintf(outbuf,"T %d %d\n\r",p0,p1);
					break;
	default:	sprintf(outbuf,"Command error\n\r");
	}
}

void four_bytes_commands(char cmd,int p0,int p1,int p2)
{
	switch ( cmd)
	{
	case 'f' 	: 	sprintf(outbuf,"f %d %d %d\n\r",p0,p1,p2);
					break;
	case 'a' 	: 	sprintf(outbuf,"a %d %d %d\n\r",p0,p1,p2);
					break;
	case 'T' 	: 	sprintf(outbuf,"T : Address %d , Speed %d , Direction %d\n\r",p0,p1,p2);
					if ( encode_throttle(p0,p1,p2) == 0 )
						insert_len3_packet();
					else
						insert_len4_packet();
					break;
	default:	sprintf(outbuf,"Command error\n\r");
	}
}

void five_bytes_commands(char cmd,int p0,int p1,int p2,int p3)
{
	switch ( cmd)
	{
	case 'T' 	: 	sprintf(outbuf,"Throttle %d %d %d %d\n\r",p0,p1,p2,p3);
					break;
	default:	sprintf(outbuf,"Command error\n\r");
	}
}

void parser(uint32_t source_buf)
{
int	p0,p1,p2,p3,p4,p5,p6,p7,pnum;
char cmd, lbuf[USBUART_BUFLEN];

	if ( source_buf == FROM_UART )
		memcpy(lbuf,&uart_rxbuf.packet[1],USBUART_BUFLEN);
	else
		memcpy(lbuf,&usb_rxbuf.packet[1],USBUART_BUFLEN);
	pnum = sscanf(lbuf,"%c %d %d %d %d %d %d %d %d",&cmd,&p0,&p1,&p2,&p3,&p4,&p5,&p6,&p7);
	switch (pnum)
	{
	case 1 :	one_byte_commands(cmd);
				break;
				/*
	case 2 :	sprintf(outbuf,"cmd : %c %d\n\r",cmd,p0);
				break;
				*/
	case 3 :	three_bytes_commands(cmd,p0,p1);
				break;
	case 4 :	four_bytes_commands(cmd,p0,p1,p2);
				break;
	case 5 :	five_bytes_commands(cmd,p0,p1,p2,p3);
				break;
	default:	sprintf(outbuf,"Command error %d\n\r",pnum);
	}


	if ( source_buf == FROM_UART )
		uart_tx_buffer((uint8_t *)(outbuf));
	else
	{
		usb_tx_buffer((uint8_t *)(outbuf));
	}
}


