///////////////////////////////////////////////////////////////////////////////
// File : main.c
// Contains: USB Power delivery debug tool firmware
//
// Written by: Jean-François DEL NERO
//
// Change History (most recent first):
///////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <string.h>

#include "buildconf.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

#include "mini_stm32f4xx_defs.h"

#include "hw_init_table.h"
#include "print.h"
#include "uart.h"

#include "goodcrc_frames.h"
#include "enc_4b5b.h"
#include "pd_crc.h"

volatile int tickcount;

typedef struct pd_frame_gen_
{
	int in_bit_size;

	int frame_user_size;
	unsigned char frame_user[1024/8];

	int frame_4b5b_size;
	unsigned char frame_4b5b[1024/4];

	int bmc_frame_size;
	unsigned char bmc_frame[1024/8];
}pd_frame_gen;

typedef struct _power_delivery_msg
{
	unsigned char sop[4];
	unsigned char header[2];
	unsigned char objs[8*4];
	uint32_t crc;

	unsigned char rev;
	unsigned char type;
	unsigned char dual_role;
	unsigned char pwr_role;
	unsigned char id;
	unsigned char extend;
	unsigned char obj_cnt;
	unsigned char good_crc;

	pd_frame_gen frm;
}power_delivery_msg;

///////////////////////////////////////////////////////////////////////////////////
//
// 4B5B Encoder
//
///////////////////////////////////////////////////////////////////////////////////

volatile unsigned char last_rx_id;

const unsigned char delaylut[]=
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x00 - 0x0F
	0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, // 0x10 - 0x1F
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, // 0x20 - 0x2F
	0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, // 0x30 - 0x3F
	0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, // 0x40 - 0x4F
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x50 - 0x5F
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x60 - 0x6F
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x70 - 0x7F
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x80 - 0x8F
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x90 - 0x9F
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xA0 - 0xAF
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xB0 - 0xBF
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xC0 - 0xCF
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xD0 - 0xDF
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xE0 - 0xEF
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // 0xF0 - 0xFF
};

static unsigned char conv_val(unsigned short val)
{
	if(val < 0x100)
	{
	  return delaylut[val];
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////
//
// Power delivery frame encoder
//
///////////////////////////////////////////////////////////////////////////////////

void encode_frame( power_delivery_msg * msg )
{
	int dst_idx;
	int i;
	uint32_t crc;

	pd_frame_gen * frm;

	frm = &msg->frm;

	dst_idx = 0;

	memset(frm->frame_4b5b,0, sizeof(frm->frame_4b5b));

	// --------------------------------------
	// Preamble
	// --------------------------------------
	for(;dst_idx < 64;dst_idx++)
	{
		if(dst_idx&1)
			frm->frame_4b5b[dst_idx>>3] |= (0x80>>(dst_idx&7));
		else
			frm->frame_4b5b[dst_idx>>3] &= ~(0x80>>(dst_idx&7));
	}

	// --------------------------------------
	// SOP
	// --------------------------------------

	// SOP   : CODE_SYNC_1->CODE_SYNC_1->CODE_SYNC_1->CODE_SYNC_2 : Communication to UFP
	// SOP'  : CODE_SYNC_1->CODE_SYNC_1->CODE_SYNC_3->CODE_SYNC_3 : Communication to USB Type-C Plug Side A
	// SOP'' : CODE_SYNC_1->CODE_SYNC_3->CODE_SYNC_1->CODE_SYNC_3 : Communication to USB Type-C Plug Side B

	// Hard Reset  : CODE_RST_1  CODE_RST_1  CODE_RST_1  CODE_RST_2   : Resets logic in all connected PD devices (UFP and/or Active/Electronically Marked Cable)
	// Cable Reset : CODE_RST_1  CODE_SYNC_1 CODE_RST_1  CODE_SYNC_3  : Reset for only Active/Electronically Marked Cable.
	// SOP’_Debug  : CODE_SYNC_1 CODE_RST_2  CODE_RST_2  CODE_SYNC_3  : Used for debug of USB Type-C Plug Side A
	// SOP’’_Debug : CODE_SYNC_1 CODE_RST_2  CODE_SYNC_3 CODE_SYNC_2  : Used for debug of USB Type-C Plug Side B

	dst_idx = encode_4b5b_quartet( frm->frame_4b5b, dst_idx, CODE_SYNC_1 );
	dst_idx = encode_4b5b_quartet( frm->frame_4b5b, dst_idx, CODE_SYNC_1 );
	dst_idx = encode_4b5b_quartet( frm->frame_4b5b, dst_idx, CODE_SYNC_1 );
	dst_idx = encode_4b5b_quartet( frm->frame_4b5b, dst_idx, CODE_SYNC_2 );

	crc = CRC_INIT;

	// --------------------------------------
	// 16 bits Header + Data / object(s)
	// --------------------------------------

	for(i=0;i<frm->frame_user_size/8;i++)
	{
		crc = pd_crc_update(crc, frm->frame_user[i]);
		dst_idx = encode_4b5b_quartet( frm->frame_4b5b, dst_idx, ( frm->frame_user[i] >> 0 ) & 0xF );
		dst_idx = encode_4b5b_quartet( frm->frame_4b5b, dst_idx, ( frm->frame_user[i] >> 4 ) & 0xF );
	}

	// --------------------------------------
	// 32 bits CRC
	// --------------------------------------

	crc = pd_crc_finalize(crc);
	for(i=0;i<4;i++)
	{
		dst_idx = encode_4b5b_quartet( frm->frame_4b5b, dst_idx, ( (crc>>(i*8)) >> 0 ) & 0xF );
		dst_idx = encode_4b5b_quartet( frm->frame_4b5b, dst_idx, ( (crc>>(i*8)) >> 4 ) & 0xF );
	}

	msg->crc = crc;
	msg->good_crc = 1;

	// --------------------------------------
	// EOP
	// --------------------------------------

	dst_idx = encode_4b5b_quartet( frm->frame_4b5b, dst_idx, CODE_EOP );

	frm->frame_4b5b_size = dst_idx;
}

///////////////////////////////////////////////////////////////////////////////////
//
// Power delivery 16 bits header
//
///////////////////////////////////////////////////////////////////////////////////

void init_header(unsigned char * buf, power_delivery_msg * msg )
{
	buf[0] = (   msg->type & 0x1F )         |  // [4..0]    : Message type (GoodCRC:0x01,GetStatus:0x12,GetSrcCap:0x7)
			 ( ( msg->dual_role & 1 )<< 5 ) |  // [5]       : Dual power role (UFP:0, DFP:1)
			 ( ( msg->rev&3 ) << 6 );          // [7..6]    : Specs revision (V1:00, V2:01, V3:10)

	buf[1] = (   msg->pwr_role & 0x1 )      |  // [8]       : Power role (Source:1, Sink:0)
			 ( ( msg->id & 7 ) << 1 )       |  // [11..9]   : Message id counter (0-7)
			 ( ( msg->obj_cnt & 7 ) << 4 )  |  // [14..12]  : Number of 32 bits objects (0-7)
			 ( ( msg->extend&1 ) << 7 );       // [15]      : Extended message
}

void decod_header(unsigned char * buf, power_delivery_msg * msg )
{
	msg->type      = (buf[0] & 0x1F);       // [4..0]    : Message type (GoodCRC:0x01,GetStatus:0x12,GetSrcCap:0x7)
	msg->dual_role = (buf[0]>>5) & 0x1;     // [5]       : Dual power role (UFP:0, DFP:1)
	msg->rev       = (buf[0]>>6) & 0x3;     // [7..6]    : Specs revision (V1:00, V2:01, V3:10)

	msg->pwr_role  = buf[1] & 0x1;          // [8]       : Power role (Source:1, Sink:0)
	msg->id        = (buf[1]>>1) & 0x7;     // [11..9]   : Message id counter (0-7)
	msg->obj_cnt   = (buf[1]>>4) & 0x7;     // [14..12]  : Number of 32 bits objects (0-7)
	msg->extend    = (buf[1]>>7) & 0x1;     // [15]      : Extended message
}

///////////////////////////////////////////////////////////////////////////////////
//
// Last stage : Biphase mark coding (BMC) encoder
//
///////////////////////////////////////////////////////////////////////////////////

void bmc_encode(pd_frame_gen * frm)
{
	int src_i;
	int dst_i;
	int state;

	dst_i = 0;
	state = 0;

	memset( frm->bmc_frame, 0, sizeof( frm->bmc_frame ));

	for(src_i=0;src_i<frm->frame_4b5b_size;src_i++)
	{
		if( frm->frame_4b5b[src_i>>3] & ( 0x80 >> (src_i&7) ) )
		{
			if(state)
				frm->bmc_frame[dst_i>>3] |= ( 0x80 >> (dst_i&7) );

			dst_i++;

			state ^= 1;

			if(state)
				frm->bmc_frame[dst_i>>3] |= ( 0x80 >> (dst_i&7) );

			dst_i++;

			state ^= 1;
		}
		else
		{
			if(state)
			{
				frm->bmc_frame[dst_i>>3] |= ( 0x80 >> (dst_i&7) );
				dst_i++;

				frm->bmc_frame[dst_i>>3] |= ( 0x80 >> (dst_i&7) );
				dst_i++;
			}
			else
			{
				dst_i += 2;
			}

			state ^= 1;
		}
	}

	while(dst_i & 7)
	{
		frm->bmc_frame[dst_i>>3] |= ( 0x80 >> (dst_i&7) );
		dst_i++;
	}

	frm->bmc_frame_size = dst_i;
}

unsigned char * bmcdecode(unsigned char * buf, unsigned char * data)
{
	int i;
	unsigned char d;

	i = 0;
	d = 0;

	while ( i < 5 )
	{
		switch(*buf++)
		{
			case 1:
				if(*buf++ == 1)
				{
					d >>= 1;
					d |= 0x10;
					i++;
				}
				else
				{
					 *data = 0xFF;
					 return buf;
				}
			break;

			case 2:
				d >>= 1;
				i++;
			break;

			default:
				 *data = 0xFF;
				 return buf;
			break;
		}
	}

	*data = d;

	return buf;
}

unsigned short * bmcdecode_tim(unsigned short * buf, unsigned char * data)
{
	int i;
	unsigned char d;

	i = 0;
	d = 0;

	while ( i < 5 )
	{
		switch(conv_val(*buf++))
		{
			case 1:
				if(conv_val(*buf++) == 1)
				{
					d >>= 1;
					d |= 0x10;
					i++;
				}
				else
				{
					 *data = 0xFF;
					 return buf;
				}
			break;

			case 2:
				d >>= 1;
				i++;
			break;

			default:
				 *data = 0xFF;
				 return buf;
			break;
		}
	}

	*data = d;

	return buf;
}

void send_frame( power_delivery_msg * msg )
{
	memset( msg->frm.frame_user,0, sizeof(msg->frm.frame_user));

	msg->frm.frame_user_size = 16 + (msg->obj_cnt*4*8);

	init_header(msg->frm.frame_user, msg);
	memcpy(msg->header,msg->frm.frame_user,2);
	memcpy(&msg->frm.frame_user[2], msg->objs,  msg->obj_cnt * 4);

	encode_frame( msg );
	bmc_encode(&msg->frm);

	return;
}

int rcv_frame( power_delivery_msg * msg , unsigned char * frame, int size )
{
	int i,j;
	unsigned char c;
	unsigned char d;
	uint32_t crc;

	c = 0x00;

	memset( msg, 0, sizeof(power_delivery_msg) );

	// sync
	i = 0;
	while( frame[i] != 2 || frame[i+1] != 1 || frame[i+2] != 1 )
	{
		i++;
	}

	while( frame[i] == 2 && frame[i+1] == 1 && frame[i+2] == 1 )
	{
		i = i + 3;
	}

	if( i > (96/2) && ( frame[i] == 2 && frame[i+1] == 2 && frame[i+2] == 2 ) ) // 0x18 -> '000'
	{
		unsigned char * ptr;
		unsigned char * endptr;
		unsigned char dat;

		dat = 0x00;

		endptr = (unsigned char *)frame + size;
		ptr = (unsigned char *)&frame[i];

		// SOP
		j = 0;
		while( ptr < endptr && dat != 0xFF && j < 4 )
		{
			ptr = bmcdecode(ptr,&dat);

			c = inv_conv_4b5b[dat];

			if( c >= 0x10 )
			{
				msg->sop[j] = c;
			}

			j++;
		}

		// Header
		j = 0;
		while( ptr < endptr && dat != 0xFF && j < 2*2 )
		{
			ptr = bmcdecode(ptr,&dat);

			d = inv_conv_4b5b[dat];

			if( d < 0x10 )
			{
				c = c >> 4;
				c |= (d << 4);

				msg->header[j>>1] = c;
			}

			j++;
		}

		msg->obj_cnt = (msg->header[1]>>4)&7;

		// Objs
		j = 0;
		c = 0;
		while( ptr < endptr && dat != 0xFF && j < (msg->obj_cnt * 4 * 2)  )
		{
			ptr = bmcdecode(ptr,&dat);

			d = inv_conv_4b5b[dat];

			if( d < 0x10 )
			{
				c = c >> 4;
				c |= (d << 4);

				msg->objs[j>>1] = c;
			}

			j++;
		}

		// crc
		unsigned char * crcptr = (unsigned char*)&msg->crc;

		j = 0;
		while( ptr < endptr && dat != 0xFF && j < 4*2 )
		{
			ptr = bmcdecode(ptr,&dat);

			d = inv_conv_4b5b[dat];

			if( d < 0x10 )
			{
				c = c >> 4;
				c |= (d << 4);

				crcptr[j>>1] = c;
			}

			j++;
		}

		decod_header((unsigned char*)&msg->header, msg );
	}

	crc = CRC_INIT;

	for(i=0;i<2;i++)
	{
		crc = pd_crc_update(crc, msg->header[i]);
	}

	for(i=0;i<msg->obj_cnt*4;i++)
	{
		crc = pd_crc_update(crc, msg->objs[i]);
	}

	crc = pd_crc_finalize(crc);

	if( crc == msg->crc )
	{
		msg->good_crc = 1;
	}

	return 0;
}

static int rcv_frame_short( unsigned short * frame, int size, unsigned char * header, unsigned char * sop  )
{
	int i,j;
	unsigned char c;
	unsigned char d;
	//uint32_t crc;

	c = 0x00;

	// sync
	i = 0;
	j = 0;
	do
	{
		c = conv_val(frame[i]);
		if(c)
		{
			j++;
		}
		else
		{
			j = 0;
		}

		i++;
	}while( j < 16 && i < size );

	if(i >= size)
		return 0;

	while( conv_val(frame[i]) != 2 || conv_val(frame[i+1]) != 1 || conv_val(frame[i+2]) != 1 )
	{
		i++;
	}

	while( conv_val(frame[i]) == 2 && conv_val(frame[i+1]) == 1 && conv_val(frame[i+2]) == 1 )
	{
		i = i + 3;
	}


	if( i > (96/2) && ( conv_val(frame[i]) == 2 && conv_val(frame[i+1]) == 2 && conv_val(frame[i+2]) == 2 ) ) // 0x18 -> '000'
	{

		unsigned short * ptr;
		unsigned short * endptr;
		unsigned char dat;

		dat = 0x00;

		endptr = (unsigned short *)frame + size;
		ptr = (unsigned short *)&frame[i];

		// SOP
		j = 0;
		while( ptr < endptr && dat != 0xFF && j < 4 )
		{
			ptr = bmcdecode_tim(ptr,&dat);

			c = inv_conv_4b5b[dat];

			if( c >= 0x10 )
			{
				sop[j] = c;
			}

			j++;
		}

		// Header
		j = 0;
		while( ptr < endptr && dat != 0xFF && j < 2*2 )
		{
			ptr = bmcdecode_tim(ptr,&dat);

			d = inv_conv_4b5b[dat];

			if( d < 0x10 )
			{
				c = c >> 4;
				c |= (d << 4);

				header[j>>1] = c;
			}

			j++;
		}
		return 1;
	}

	return 0;
}

void print_msg( power_delivery_msg * msg )
{
	int i;

	for(i=0;i<4;i++)
	{
		switch( msg->sop[i] )
		{
			case CODE_SYNC_1:
				print((unsigned char*)"SYNC_1 ");
			break;
			case CODE_SYNC_2:
				print((unsigned char*)"SYNC_2 ");
			break;
			case CODE_RST_1:
				print((unsigned char*)"RST_1 ");
			break;
			case CODE_RST_2:
				print((unsigned char*)"RST_2 ");
			break;
			case CODE_EOP:
				print((unsigned char*)"EOP ");
			break;
			case CODE_SYNC_3:
				print((unsigned char*)"SYNC_3 ");
			break;
			default:
				//print((unsigned char*)"Unknow code : 0x%X\n", msg->sop[i]);
			break;
		}
	}

	print((unsigned char*)"[ ");

	for(i=0;i<2;i++)
	{
		printhex(msg->header[i]);
		print((unsigned char*)" ");
	}
	print((unsigned char*)"] ");

	print((unsigned char*)"[ ");

	for(i=0;i<msg->obj_cnt * 4;i++)
	{
		printhex(msg->objs[i]);
		print((unsigned char*)" ");
	}

	print((unsigned char*)"] ");

	printhex_long(msg->crc);
	print((unsigned char*)" ");

	if( msg->good_crc )
	{
		print((unsigned char*)"(Valid CRC)" );
	}
	else
	{
		print((unsigned char*)"(Bad CRC)" );
	}

	print((unsigned char*)" (msg_type:");
	printhex(msg->type);
	print((unsigned char*)" dualrole:");
	printhex(msg->dual_role);
	print((unsigned char*)" pwr_role:");
	printhex(msg->pwr_role);
	print((unsigned char*)" rev:");
	printhex(msg->rev);
	print((unsigned char*)" msg_id:");
	printhex(msg->id);
	print((unsigned char*)" extend:");
	printhex(msg->extend);
	print((unsigned char*)" objcnt:");
	printhex(msg->obj_cnt);
	print((unsigned char*)")");
	print((unsigned char*)"\n");

	for(int i=0;i<msg->frm.bmc_frame_size;i++)
	{
		if( msg->frm.bmc_frame[i>>3] & ( 0x80 >> (i&7) ) )
		{
			print((unsigned char*)"1");
		}
		else
		{
			print((unsigned char*)"0");
		}
	}
	print((unsigned char*)"\n");
}

volatile unsigned char tx_frame_buffer[512] __attribute__((aligned(4)));

#define RX_BUFFER_SIZE (360*8*2)

#define RX_BUFFER_MAX 8

unsigned short rx_frame_buffer_buf0[RX_BUFFER_SIZE] __attribute__((aligned(4)));
unsigned short rx_frame_buffer_buf1[RX_BUFFER_SIZE] __attribute__((aligned(4)));
unsigned short rx_frame_buffer_buf2[RX_BUFFER_SIZE] __attribute__((aligned(4)));
unsigned short rx_frame_buffer_buf3[RX_BUFFER_SIZE] __attribute__((aligned(4)));
unsigned short rx_frame_buffer_buf4[RX_BUFFER_SIZE] __attribute__((aligned(4)));
unsigned short rx_frame_buffer_buf5[RX_BUFFER_SIZE] __attribute__((aligned(4)));

unsigned short rx_frame_buffer_buf6[RX_BUFFER_SIZE] __attribute__((aligned(4)));
unsigned short rx_frame_buffer_buf7[RX_BUFFER_SIZE] __attribute__((aligned(4)));

unsigned char  rx_frame_buffer_process[RX_BUFFER_SIZE] __attribute__((aligned(4)));

const unsigned short * rx_buffers[RX_BUFFER_MAX]=
{
	rx_frame_buffer_buf0,
	rx_frame_buffer_buf1,
	rx_frame_buffer_buf2,
	rx_frame_buffer_buf3,
	rx_frame_buffer_buf4,
	rx_frame_buffer_buf5,
	rx_frame_buffer_buf6,
	rx_frame_buffer_buf7
};

volatile unsigned int rx_buffers_size[RX_BUFFER_MAX];

volatile int cur_rx_frame_buffer;
volatile int next_process_frame_buffer;

volatile unsigned char tx_dma_buffer[FULL_DMATX_SIZE] __attribute__((aligned(4)));
volatile unsigned short rx_dma_buffer[FULL_DMARX_SIZE] __attribute__((aligned(4)));

volatile unsigned int auto_goodcrc;
volatile unsigned int pwr_role;
volatile unsigned int dual_role;
volatile unsigned int rev;
volatile unsigned int goodcrcbase;

typedef struct frame_receiver_
{
	volatile int receiving;
	int size;
	int rsize;
	volatile int idx;
	volatile int update;
}frame_receiver;

typedef struct frame_sender_
{
	volatile int size;
	volatile unsigned char * bufptr;

	volatile int rsize;
	volatile unsigned char * rbufptr;

	int idx;
	volatile int update;
}frame_sender;

volatile frame_sender frm_snd;
frame_receiver frm_rcv;

void SysTick_Handler(void)
{
	tickcount++;
}

const unsigned char __attribute__((aligned(4))) enc[4]=
{
	0x00,
	0x0F,
	0xF0,
	0xFF
};

void DMA_IRQ_ENTRY_NAME_TX(void)
{
	unsigned char c;
	unsigned char * ptr;
	unsigned char * ptrtx;

	int i;
	int cnt;

	if(frm_snd.update)
	{
		frm_snd.rbufptr = frm_snd.bufptr;
		frm_snd.rsize = frm_snd.size;
		frm_snd.idx = 0;
		frm_snd.update = 0;
	}

	if (CONFIG_DMA_CTRL_TX->HISR & (DMA_HISR_TCIF5) )
	{
		CONFIG_DMA_CTRL_TX->HIFCR |= (DMA_HIFCR_CTCIF5);

		ptr = (unsigned char *)&tx_dma_buffer[(FULL_DMATX_SIZE/2)];
		ptrtx = (unsigned char *)frm_snd.rbufptr + frm_snd.idx;

		cnt = frm_snd.rsize - frm_snd.idx;

		if( cnt > (FULL_DMATX_SIZE/2)/4)
			cnt = (FULL_DMATX_SIZE/2)/4;

		for(i=0;i<cnt;i++)
		{
			c = *ptrtx++;

			*ptr++ = enc[(c>>6)];
			*ptr++ = enc[(c>>4)&3];
			*ptr++ = enc[(c>>2)&3];
			*ptr++ = enc[c&3];
		}

		frm_snd.idx += cnt;

		for(;i<(FULL_DMATX_SIZE/2)/4;i++)
		{
			*ptr++ = 0xFF;
			*ptr++ = 0xFF;
			*ptr++ = 0xFF;
			*ptr++ = 0xFF;
		}
	}

	if (CONFIG_DMA_CTRL_TX->HISR & (DMA_HISR_HTIF5) )
	{
		CONFIG_DMA_CTRL_TX->HIFCR |= (DMA_HIFCR_CHTIF5);

		ptr = (unsigned char *)&tx_dma_buffer[0];
		ptrtx = (unsigned char *)frm_snd.rbufptr + frm_snd.idx;

		cnt = frm_snd.rsize - frm_snd.idx;

		if( cnt > (FULL_DMATX_SIZE/2)/4)
			cnt = (FULL_DMATX_SIZE/2)/4;

		for(i=0;i<cnt;i++)
		{
			c = *ptrtx++;

			*ptr++ = enc[(c>>6)];
			*ptr++ = enc[(c>>4)&3];
			*ptr++ = enc[(c>>2)&3];
			*ptr++ = enc[c&3];
		}

		frm_snd.idx += cnt;

		for(;i<(FULL_DMATX_SIZE/2)/4;i++)
		{
			*ptr++ = 0xFF;
			*ptr++ = 0xFF;
			*ptr++ = 0xFF;
			*ptr++ = 0xFF;
		}
	}

	if (CONFIG_DMA_CTRL_TX->HISR & (DMA_HISR_TEIF5) )
	{
		CONFIG_DMA_CTRL_TX->HIFCR |= (DMA_HIFCR_CTEIF5);
	}
}

void DMA_IRQ_ENTRY_NAME_RX(void)
{
	int i;
	int curbuf;
	unsigned short * tmp_short_ptr;
	unsigned short * tmp_short_ptr2;

	EXEC_DIS_IRQ(EXTI15_10_IRQn);

	if (CONFIG_DMA_CTRL_RX->LISR & (DMA_LISR_TCIF1) )
	{
		CONFIG_DMA_CTRL_RX->LIFCR |= (DMA_LIFCR_CTCIF1);

		curbuf = cur_rx_frame_buffer;
		tmp_short_ptr = (unsigned short *)&rx_dma_buffer[(FULL_DMARX_SIZE/2)];
		tmp_short_ptr2 =(unsigned short * )rx_buffers[curbuf];

		if( rx_buffers_size[curbuf] < RX_BUFFER_SIZE - (FULL_DMARX_SIZE/2))
		{
			tmp_short_ptr2 += rx_buffers_size[curbuf];

			for(i=0;i<(FULL_DMARX_SIZE/2)/4;i++)
			{
				*tmp_short_ptr2++ = *tmp_short_ptr++;
				*tmp_short_ptr2++ = *tmp_short_ptr++;
				*tmp_short_ptr2++ = *tmp_short_ptr++;
				*tmp_short_ptr2++ = *tmp_short_ptr++;
			}

			rx_buffers_size[curbuf] += (FULL_DMARX_SIZE/2);
		}
	}

	if (CONFIG_DMA_CTRL_RX->LISR & (DMA_LISR_HTIF1) )
	{
		CONFIG_DMA_CTRL_RX->LIFCR |= (DMA_LIFCR_CHTIF1);

		curbuf = cur_rx_frame_buffer;
		tmp_short_ptr = (unsigned short * )&rx_dma_buffer[0];
		tmp_short_ptr2 = (unsigned short *)rx_buffers[curbuf];

		if( rx_buffers_size[curbuf] < RX_BUFFER_SIZE - (FULL_DMARX_SIZE/2))
		{
			tmp_short_ptr2 += rx_buffers_size[curbuf];

			for(i=0;i<(FULL_DMARX_SIZE/2)/4;i++)
			{
				*tmp_short_ptr2++ = *tmp_short_ptr++;
				*tmp_short_ptr2++ = *tmp_short_ptr++;
				*tmp_short_ptr2++ = *tmp_short_ptr++;
				*tmp_short_ptr2++ = *tmp_short_ptr++;
			}

			rx_buffers_size[curbuf] += (FULL_DMARX_SIZE/2);
		}
	}

	if (CONFIG_DMA_CTRL_RX->LISR & (DMA_LISR_TEIF1) )
	{
		CONFIG_DMA_CTRL_RX->LIFCR |= (DMA_LIFCR_CTEIF1);
	}

	EXEC_EN_IRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
	unsigned int strt,end,rem,i;
	unsigned char header[2];
	unsigned char sop[4];

	unsigned short * tmp_short_ptr;
	unsigned short * tmp_short_ptr2;
	int curbuf,process;

	process = 0;
	curbuf = cur_rx_frame_buffer;

	if( rx_buffers_size[curbuf] < RX_BUFFER_SIZE - (FULL_DMARX_SIZE/2))
	{
		if(rx_buffers_size[curbuf] < 64 )
		{
			// Glitch ... just discard it !
			rx_buffers_size[curbuf] = 0;
			__DSB();

			tmp_short_ptr = (unsigned short *)&rx_dma_buffer[0];
			for(i=0;i<(FULL_DMARX_SIZE)/4;i++)
			{
				*tmp_short_ptr++ = 0xFFFF;
				*tmp_short_ptr++ = 0xFFFF;
				*tmp_short_ptr++ = 0xFFFF;
				*tmp_short_ptr++ = 0xFFFF;
			}
		}
		else
		{
			// next buffer !
			rx_buffers_size[(cur_rx_frame_buffer + 1) % (RX_BUFFER_MAX)] = 0;
			cur_rx_frame_buffer = (cur_rx_frame_buffer + 1) % (RX_BUFFER_MAX);
			__DSB();

			tmp_short_ptr = (unsigned short *)&rx_dma_buffer[0];
			tmp_short_ptr2 = (unsigned short *)rx_buffers[curbuf];

			tmp_short_ptr2 += rx_buffers_size[curbuf];

			rem = CONFIG_DMA_CHN_RX_CTRL->NDTR;// = FULL_DMARX_SIZE;

			if( rem >= FULL_DMARX_SIZE/2 )
			{
				rem -= (FULL_DMARX_SIZE/2);
				strt = 0;
				end =  ((FULL_DMARX_SIZE/2) - rem);
			}
			else
			{
				strt = (FULL_DMARX_SIZE/2);
				end =  ((FULL_DMARX_SIZE) - rem);
			}

			for(int i=strt;i<end;i++)
			{
				*tmp_short_ptr2++ = tmp_short_ptr[i];
			}

			rx_buffers_size[curbuf] += (end - strt);

			tmp_short_ptr = (unsigned short *)&rx_dma_buffer[0];
			for(i=0;i<(FULL_DMARX_SIZE)/4;i++)
			{
				*tmp_short_ptr++ = 0xFFFF;
				*tmp_short_ptr++ = 0xFFFF;
				*tmp_short_ptr++ = 0xFFFF;
				*tmp_short_ptr++ = 0xFFFF;
			}

			process = 1;
		}

		__DSB();
	}
	else
	{
		// next buffer !
		rx_buffers_size[(cur_rx_frame_buffer + 1) % (RX_BUFFER_MAX)] = 0;
		cur_rx_frame_buffer = (cur_rx_frame_buffer + 1) % (RX_BUFFER_MAX);
		__DSB();
	}

	if( process )
	{
		int size;

		tmp_short_ptr2 = (unsigned short *)rx_buffers[curbuf];
		size = rx_buffers_size[curbuf];

		if( auto_goodcrc )
		{
			if ( rcv_frame_short( tmp_short_ptr2, size, (unsigned char*)&header, (unsigned char*)sop ) )
			{
				last_rx_id = (header[1]>>1)&7;

				// If not a GoodCRC or is a Data message ... Send the goodcrc message
				if( ( ( header[0] & 0x1F ) != 0x01 ) || ( (header[1]>>4) & 0x7 ) )
				{
                    if( (sop[0] == CODE_SYNC_1) && (sop[1] == CODE_SYNC_1) && (sop[2] == CODE_SYNC_1) && (sop[3] == CODE_SYNC_2) )
                    {
					    frm_snd.bufptr = (unsigned char *)&goodcrcfrm[goodcrcbase + ((last_rx_id&7)*GOODSRC_FRAME_SIZE)];
					    frm_snd.size = GOODSRC_FRAME_SIZE;
					    frm_snd.update = 1;
                    }
				}

				//printchar('K');
			}
		}
	}

	EXTI->PR = EXTI_PR_PR10;

	__DSB();
	__asm("nop\n");
}

void set_led(int state)
{
	if(state)
		GPIOB->BSRR = (0x0001) << (PIN_3);
	else
		GPIOB->BSRR = (0x0001) << (PIN_3+16);
}

int alignbyte(int cnt)
{
	if(cnt&7)
		return ((cnt>>3) + 1);
	else
		return (cnt>>3);
}


void start_dma()
{
	int i;

	for(i=0;i<FULL_DMATX_SIZE;i++)
	{
		tx_dma_buffer[i] = 0xFF;
	}

	for(i=0;i<FULL_DMARX_SIZE;i++)
	{
		rx_dma_buffer[i] = 0x0000;
	}

	CONFIG_DMA_CHN_TX_CTRL->M0AR  = (uint32_t)&tx_dma_buffer;
	CONFIG_DMA_CHN_TX_CTRL->NDTR = sizeof(tx_dma_buffer);
	CONFIG_DMA_CHN_TX_CTRL->PAR  = (uint32_t)&CONFIG_SPI_CTRL->DR;

	CONFIG_DMA_CHN_RX_CTRL->M0AR  = (uint32_t)&rx_dma_buffer;
	CONFIG_DMA_CHN_RX_CTRL->NDTR = FULL_DMARX_SIZE;
	CONFIG_DMA_CHN_RX_CTRL->PAR  = (uint32_t)&TIM1->CCR1;

	__DSB();

	//CONFIG_SPI_CTRL->CR2 |= (SPI_CR2_RXDMAEN);
	TIM1->CNT = 0;
	TIM1->DIER |= (uint16_t)(TIM_DMA_CC1);
	TIM1->CCER |= TIM_CCER_CC1E;
	//TIM1->DIER |= TIM_IT_UPDATE;  // IT enabled

	__DSB();

	CONFIG_DMA_CHN_RX_CTRL->CR  |= DMA_SxCR_EN; // Ch2 enabled
	__DSB();

	CONFIG_DMA_CHN_TX_CTRL->CR  |= DMA_SxCR_EN; // Ch3 enabled
	__DSB();

	CONFIG_SPI_CTRL->CR2 |= (SPI_CR2_TXDMAEN);
	__DSB();

	CONFIG_SPI_CTRL->CR1 |= SPI_CR1_SPE;
	__DSB();
}

unsigned int get_ms_tick()
{
	return tickcount; // SYST_CVR
}

unsigned int get_elapsed_ms_tick(unsigned int start)
{
	unsigned int curcount;

	curcount = tickcount;

	if( curcount < start )
	{
		return ((0xFFFFFFFF - start) + curcount);
	}
	else
	{
		return (curcount - start);
	}
}

void mswait(unsigned int ms)
{
	unsigned int start,total_time;

	total_time = ms;
	start = tickcount;
	while(get_elapsed_ms_tick(start) < total_time)
	{
		__WFI();
	}
}

void decode_frame(frame_receiver * rcv, unsigned char * rx_buf)
{

}

int main(void)
{
	unsigned char *  tmp_char_ptr;
	unsigned short * tmp_short_ptr2;
	uint32_t cmd,cmdidx;
	unsigned char cfg;

	unsigned char c,serchar;
	int wr_idx,i,size,s,rx_frame;
	//power_delivery_msg rxfrm;

	last_rx_id = 0;

	auto_goodcrc = 0;
	rx_frame = 1;

	cfg = 0xB;
	pwr_role = (cfg >> 0) & 1;
	dual_role = (cfg >> 1) & 1;
	rev = (cfg >> 2) & 3;
	goodcrcbase = (GOODSRC_FRAME_SIZE * 8) * (cfg&0xF);

	pwr_role = 0;
	dual_role = 0;
	rev = 2;

	frm_snd.update = 0;
	frm_snd.rsize = 0;
	frm_snd.bufptr = tx_frame_buffer;
	frm_snd.idx = 0;

	frm_rcv.receiving = 0;
	frm_rcv.update = 0;
	frm_rcv.rsize = 0;
	frm_rcv.idx = 0;

	cur_rx_frame_buffer = 0;
	next_process_frame_buffer = 0;
	memset((int*)rx_buffers_size,0,sizeof(rx_buffers_size));

	tickcount = 0;
	exec_hw_init_table(HW_INIT_GLOBAL_INIT);
	__enable_irq();

GPIOC->BSRR = (0x0001) << (3);
	start_dma();
GPIOC->BSRR = (0x0001) << (16+3);
	tickcount = 0;

	print((unsigned char*)"Ready\n");


#if 0
	for(i =0;i<8;i++)
	{
		print((unsigned char*)"F ");
		printhex(i);
		print((unsigned char*)" ");

		tmp_char_ptr = goodcrcframes[i];

		wr_idx = 0;
		for(wr_idx=0;wr_idx<GOODSRC_FRAME_SIZE*8;wr_idx++)
		{
			if(tmp_char_ptr[(wr_idx>>3)] & (0x80>>(wr_idx&7)))
				print("1");
			else
				print("0");
		}
		print("\n\n");
	}
#endif

// SEND:HEXBUFFER\n
// RECV:<TSTAMPHEX>HEXBUFFER\n
// GCRC:ON/OFF
// CRC0
// CFMX
// RXON
// RXOF
// ACKN\n
// PING\n

	cmdidx = 0;
	cmd = 0;
	wr_idx = 0;

	for(;;)
	{
		do
		{
			do
			{
				__WFI();
				serchar = popserchar();
			}while(serchar == 0xFF && (cur_rx_frame_buffer == next_process_frame_buffer) );

			// RX frame
			while((cur_rx_frame_buffer != next_process_frame_buffer))
			{
				if(rx_frame)
				{
					tmp_char_ptr = rx_frame_buffer_process;
					tmp_short_ptr2 = (unsigned short *)rx_buffers[next_process_frame_buffer];
					size = rx_buffers_size[next_process_frame_buffer];

					s = 0;
					for(i=0;i<size;i++)
					{
						c = conv_val(*tmp_short_ptr2++);
						if(c)
						{
							*tmp_char_ptr++ = c;
							s++;
						}
					}

					print((unsigned char*)"RECV");

					tmp_char_ptr = rx_frame_buffer_process;
					unsigned char state = 0;
					unsigned char d = 0x01;
					int quartetcnt = 0;
					for(i=0;i<s;i++)
					{
						c = *tmp_char_ptr++;

						while(c--)
						{
							d <<= 1;
							d |= state;
							if( d & 0x10 )
							{
								// print quartet
								printhexquartet(d);
								quartetcnt++;
								d = 0x01;
							}
						}

						state ^= 0x1;
					}

					if( d != 0x01)
					{
						state = 0x1;
						// Padding
						while( !(d & 0x10 ) )
						{
								d <<= 1;
								d |= state;
						}
						printhexquartet(d);
						quartetcnt++;
					}

					if(quartetcnt&1)
						printchar('F');

					print((unsigned char*)"\n");
				}

				//rcv_frame( &rxfrm , rx_frame_buffer_process, s );
				//print_msg( &rxfrm );

				next_process_frame_buffer = ((next_process_frame_buffer + 1) % (RX_BUFFER_MAX));
			}

			if( serchar != 0xFF )
			{
				if( cmdidx < 4)
				{
					wr_idx = 0;

					if( serchar != '\n' )
					{
						tmp_char_ptr = (unsigned char*)&cmd;
						tmp_char_ptr[cmdidx++] = serchar;
					}
					else
						cmdidx = 0;
				}
				else
				{
					switch(cmd)
					{
						case 0x444E4553: // SEND
							if( serchar=='\n')
							{
								cmdidx = 0;
								// send
								if(wr_idx&1)
								{
									tx_frame_buffer[(wr_idx>>1)] <<= 4;
									tx_frame_buffer[(wr_idx>>1)] |= 0xF;
									wr_idx++;
								}

								frm_snd.bufptr = tx_frame_buffer;
								frm_snd.size = wr_idx/2;
								frm_snd.update = 1;
								//print((unsigned char*)"ACKN\n");
							}
							else
							{
								if( (wr_idx / 2) < sizeof(tx_frame_buffer)  )
								{
									tx_frame_buffer[(wr_idx>>1)] <<= 4;
									tx_frame_buffer[(wr_idx>>1)] |= hex2quartet(serchar);
									wr_idx++;
								}
							}
						break;

						case 0x474E4950: // PING
							print((unsigned char*)"PONG\n");
							cmdidx = 0;
						break;

						case 0x30435243: // CRC0
							auto_goodcrc = 0;
							//print((unsigned char*)"ACKN\n");
							cmdidx = 0;
						break;

						case 0x31435243: // CRC1
							auto_goodcrc = 1;
							//print((unsigned char*)"ACKN\n");
							cmdidx = 0;
						break;

						case 0x4D474643: // CFGM
							if( serchar=='\n')
							{
								cmdidx = 0;
							}
							else
							{
								cfg = hex2quartet(serchar) & 0xF;
								pwr_role = (cfg >> 0) & 1;
								dual_role = (cfg >> 1) & 1;
								rev = (cfg >> 2) & 3;
								goodcrcbase = (GOODSRC_FRAME_SIZE * 8) * (cfg&0xF);
							}
						break;

						case 0x464F5852: // RXOF
							rx_frame = 0;
							cmdidx = 0;
						break;

						case 0x4E4F5852: // RXON
							rx_frame = 1;
							cmdidx = 0;
						break;

						default:
							print((unsigned char*)"NACK\n");
							cmdidx = 0;
						break;
					}
				}
			}

		}while( 1 );
	}

	return 0;
}
