///////////////////////////////////////////////////////////////////////////////
// File : pd_crc.c
// Contains: USB Power delivery CRC
//
// Written by: Jean-François DEL NERO
//
// Change History (most recent first):
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
//
// Power delivery CRC32
//
// (as in example code from USB_PD_R3_2 V1.0 2023-10.pdf)
//
///////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include "pd_crc.h"

uint32_t pd_crc_update(uint32_t crc, uint8_t data)
{
	uint32_t newbit, newword, rl_crc;

	for(int i=0; i<8; i++)
	{
		newbit = ( ( crc>>31 ) ^ ( ( data>>i ) & 1 ) ) & 1;

		if(newbit)
			newword = CRC_POLY-1;
		else
			newword = 0;

		rl_crc = (crc<<1) | newbit;
		crc = rl_crc ^ newword;
	}

	return crc;
}

uint32_t pd_crc_finalize(uint32_t crc)
{
	int ret = 0;
	int j, bit;

	crc = ~crc;
	for(int i=0;i<32;i++)
	{
		j = 31-i;
		bit = ( crc>>i ) & 1;
		ret |= ( bit<<j );
	}
	return ret;
}
