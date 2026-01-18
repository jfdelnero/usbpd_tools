///////////////////////////////////////////////////////////////////////////////
// File : enc_4b5b.c
// Contains: USB Power delivery 4b5b encoder / decoder
//
// Written by: Jean-François DEL NERO
//
// Change History (most recent first):
///////////////////////////////////////////////////////////////////////////////

#include <stdint.h>

#include "enc_4b5b.h"

const unsigned char conv_4b5b[]=
{
	// 0x0 <> 0xF
	0x1E, 0x09, 0x14, 0x15,
	0x0A, 0x0B, 0x0E, 0x0F,
	0x12, 0x13, 0x16, 0x17,
	0x1A, 0x1B, 0x1C, 0x1D,

	0x18, // SYNC-1
	0x11, // SYNC-2
	0x07, // RST-1
	0x19, // RST-2
	0x0D, // EOP
	0x06  // SYNC3
};

const unsigned char inv_conv_4b5b[]=
{
		 0x00,         0x00,         0x00,         0x00,  // 0x00 - 0x03
		 0x00,         0x00,  CODE_SYNC_3,   CODE_RST_1,  // 0x04 - 0x07
		 0x00,         0x01,         0x04,         0x05,  // 0x08 - 0x0B
		 0x00,     CODE_EOP,         0x06,         0x07,  // 0x0C - 0x0F
		 0x00,  CODE_SYNC_2,         0x08,         0x09,  // 0x10 - 0x17
		 0x02,         0x03,         0x0A,         0x0B,  // 0x14 - 0x1B
  CODE_SYNC_1,   CODE_RST_2,         0x0C,         0x0D,  // 0x18 - 0x1B
		 0x0E,         0x0F,         0x00,         0x00   // 0x1C - 0x1F
};


int encode_4b5b_quartet(unsigned char * dstbuf, int idx, unsigned char quartet)
{
	unsigned char code5b;
	int i;

	code5b = conv_4b5b[quartet];

	for(i=0;i<5;i++)
	{
		if( code5b & (0x01<<i) )
		{
			dstbuf[idx>>3] |= (0x80 >> (idx&7));
		}
		else
		{
			dstbuf[idx>>3] &= ~(0x80 >> (idx&7));
		}

		idx++;
	}

	return idx;
}
