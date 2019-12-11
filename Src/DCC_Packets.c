/*
 * DCC_Packets.c
 *
 *  Created on: Dec 9, 2019
 *      Author: fil
 */


#include "main.h"
#include <string.h>

extern	s_len3_packet	idle_packet;
extern	s_len3_packet	reset_packet;
extern	s_main_track 	main_track;
extern	s_len3_packet 	len3_packet;
extern	s_len4_packet 	len4_packet;
extern	s_len5_packet 	len5_packet;
extern	s_len6_packet 	len6_packet;
extern	uint32_t 		*dcc_packet;
extern	uint32_t		dcc_packet_size;
extern	uint32_t		update_semaphore;
extern	void packet_callback(void);
extern	void packet_callback(void);

uint16_t *mirror_packet;
void encode_byte(uint16_t *dest , uint8_t value)
{
	uint8_t i,mask=0x80;
	for ( i=0;i<8;i++)
	{
		dest[i] = BIT_0;
		if (( value & mask) == mask)
			dest[i] = BIT_1;
		mask = mask >> 1;
	}
}

uint32_t encode_throttle(int cab,int speed,int direction)
{
	// 01DCSSSS
	int cab1,cab2,command,errcheck, retval=0;

	command = 0x40 | (direction << 5) | (speed & 0x1f);

	if ( cab > 127 )
	{
		retval=1;
		cab1 = (cab >> 8 ) | 0xc0;
		cab2 = cab & 0xff;
		errcheck = cab1 ^ cab2 ^ command;
		encode_byte(len4_packet.byte1,cab1);
		encode_byte(len4_packet.byte2,cab2);
		encode_byte(len4_packet.byte3,command);
		encode_byte(len4_packet.errcheck,errcheck);
		mirror_packet = (uint16_t *)&len4_packet;
	}
	else
	{
		retval=0;
		errcheck = cab ^ command;
		encode_byte(len3_packet.byte1,cab);
		encode_byte(len3_packet.byte2,command);
		encode_byte(len3_packet.errcheck,errcheck);
		mirror_packet = (uint16_t *)&len3_packet;
	}
	return retval;
}

void init_bufs(void)
{
uint32_t	i;
	/* Power ON */
	main_track.main_power = 1;
	/* Prepares idle packets */
	for(i=0;i<PREAMBLE_LEN;i++)
		idle_packet.preamble[i] = BIT_1;
	encode_byte(idle_packet.byte1,0xff);
	encode_byte(idle_packet.byte2,0x00);
	encode_byte(idle_packet.errcheck,0xff);
	idle_packet.byte1_start=BIT_0;
	idle_packet.byte2_start=BIT_0;
	idle_packet.errdet_start=BIT_0;
	for(i=0;i<CLOSER_LEN;i++)
		idle_packet.closer[i] = BIT_1;

	/* Prepares reset packets */
	for(i=0;i<PREAMBLE_LEN;i++)
		reset_packet.preamble[i] = BIT_1;
	encode_byte(reset_packet.byte1,0x00);
	encode_byte(reset_packet.byte2,0x00);
	encode_byte(reset_packet.errcheck,0x00);
	reset_packet.byte1_start=BIT_0;
	reset_packet.byte2_start=BIT_0;
	reset_packet.errdet_start=BIT_0;
	for(i=0;i<CLOSER_LEN;i++)
		reset_packet.closer[i] = BIT_1;

	/* Prepare all the others */
	for(i=0;i<PREAMBLE_LEN;i++)
	{
		len3_packet.preamble[i] = BIT_1;
		len4_packet.preamble[i] = BIT_1;
		len5_packet.preamble[i] = BIT_1;
		len6_packet.preamble[i] = BIT_1;
	}
	for(i=0;i<CLOSER_LEN;i++)
	{
		len3_packet.closer[i] = BIT_1;
		len4_packet.closer[i] = BIT_1;
		len5_packet.closer[i] = BIT_1;
		len6_packet.closer[i] = BIT_1;
	}

	encode_byte(len3_packet.byte1,0xff);
	encode_byte(len3_packet.byte2,0x00);
	encode_byte(len3_packet.errcheck,0xff);
	len3_packet.byte1_start=BIT_0;
	len3_packet.byte2_start=BIT_0;
	len3_packet.errdet_start=BIT_0;

	encode_byte(len4_packet.byte1,0xff);
	encode_byte(len4_packet.byte2,0x00);
	encode_byte(len4_packet.byte3,0x00);
	encode_byte(len4_packet.errcheck,0xff);
	len4_packet.byte1_start=BIT_0;
	len4_packet.byte2_start=BIT_0;
	len4_packet.byte3_start=BIT_0;
	len4_packet.errdet_start=BIT_0;

	encode_byte(len5_packet.byte1,0xff);
	encode_byte(len5_packet.byte2,0x00);
	encode_byte(len5_packet.byte3,0x00);
	encode_byte(len5_packet.byte4,0x00);
	encode_byte(len5_packet.errcheck,0xff);
	len5_packet.byte1_start=BIT_0;
	len5_packet.byte2_start=BIT_0;
	len5_packet.byte3_start=BIT_0;
	len5_packet.byte4_start=BIT_0;
	len5_packet.errdet_start=BIT_0;

	encode_byte(len6_packet.byte1,0xff);
	encode_byte(len6_packet.byte2,0x00);
	encode_byte(len6_packet.byte3,0x00);
	encode_byte(len6_packet.byte4,0x00);
	encode_byte(len6_packet.byte5,0x00);
	encode_byte(len6_packet.errcheck,0xff);
	len6_packet.byte1_start=BIT_0;
	len6_packet.byte2_start=BIT_0;
	len6_packet.byte3_start=BIT_0;
	len6_packet.byte4_start=BIT_0;
	len6_packet.byte5_start=BIT_0;
	len6_packet.errdet_start=BIT_0;
}

void insert_reset_packet(void)
{
	debug_flag_pulse();
	if ( main_track.main_power == 0)
		return;
	while ( update_semaphore == 0);
	dcc_packet = (uint32_t *)&reset_packet ;
	dcc_packet_size = sizeof(idle_packet)/2;
	debug_flag_pulse();
}

void insert_len3_packet(void)
{
	debug_flag_pulse();
	if ( main_track.main_power == 0)
		return;
	while ( update_semaphore == 0);
	dcc_packet = (uint32_t *)&len3_packet ;
	dcc_packet_size = sizeof(idle_packet)/2;
	debug_flag_pulse();
}

void insert_len4_packet(void)
{
	debug_flag_pulse();
	if ( main_track.main_power == 0)
		return;
	while ( update_semaphore == 0);
	dcc_packet = (uint32_t *)&len4_packet ;
	dcc_packet_size = sizeof(s_len4_packet)/2;
	debug_flag_pulse();

}

