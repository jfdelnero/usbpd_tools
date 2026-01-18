///////////////////////////////////////////////////////////////////////////////////
// File : pd_frame_gen.c
// Contains: Power delivery frame bitstream generator
//
// Written by: Jean-François DEL NERO
///////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <unistd.h>

#include "serial.h"
#include "pd_defs.h"

typedef struct pd_frame_gen_
{
	int in_bit_size;

	int frame_user_size;
	unsigned char frame_user[1024];

	int frame_4b5b_size;
	unsigned char frame_4b5b[1024];

	int bmc_frame_size;
	unsigned char bmc_frame[1024];
}pd_frame_gen;

typedef struct _power_delivery_msg
{
	uint32_t sop;
	unsigned char header[2];
	unsigned char ext_header[2];
	unsigned char objs[2+260];
	uint32_t crc;

	unsigned char rev;
	unsigned short type;
	unsigned char dual_role;
	unsigned char pwr_role;
	unsigned char id;
	unsigned char extend;
	unsigned char obj_cnt;
	unsigned char good_crc;

	// extended
	unsigned char chunked;
	unsigned char chunk_number;
	unsigned char request_chunk;
	unsigned short data_size;
	//

	pd_frame_gen frm;
}power_delivery_msg;


typedef struct app_ctx_
{
	int serial;

	int protocolrev;
	int dual_role;
	int pwr_role;

	int id_cnt;

	uint32_t source_caps[7];
	int      source_caps_cnt;

	uint32_t sink_caps[7];
	int      sink_caps_cnt;
}app_ctx;

void printhelp()
{
	fprintf(stdout, "Options:\n");
	fprintf(stdout, "  -help \t\t\t: This help\n");
	fprintf(stdout, "  -msg_type:[type id]\t\t: Message type ID (GoodCRC:0x01,GetStatus:0x12,GetSrcCap:0x7...)\n");
	fprintf(stdout, "  -dualrole:[0/1]\t\t: Dual role flag (UFP:0, DFP:1)\n");
	fprintf(stdout, "  -pwrrole:[0/1]\t\t: Power role flag (Source:1, Sink:0)\n");
	fprintf(stdout, "  -rev:[0/1/2/3]\t\t: Protocol revision flags (V1:0, V2:1, V3:2)\n");
	fprintf(stdout, "  -msg_id:[0-7]\t\t\t: Frame counter/ID\n");
	fprintf(stdout, "  -extend:[0/1]\t\t\t: Extended frame flag\n");
	fprintf(stdout, "  -objs:[0102030408090A0B]\t: Objects data\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "Message Type list:\n");
	fprintf(stdout, "GoodCRC=1\n");
	fprintf(stdout, "GoToMin=2\n");
	fprintf(stdout, "Accept=3\n");
	fprintf(stdout, "Reject=4\n");
	fprintf(stdout, "Ping=5\n");
	fprintf(stdout, "PS_RDY=6\n");
	fprintf(stdout, "GetSourceCap=7\n");
	fprintf(stdout, "GetSinkCap=8\n");
	fprintf(stdout, "Dr Swap=9\n");
	fprintf(stdout, "Pr Swap=10\n");
	fprintf(stdout, "Vconn Swap=11\n");
	fprintf(stdout, "Wait=12\n");
	fprintf(stdout, "Soft Reset=13\n");

	fprintf(stdout, "\n");
}

///////////////////////////////////////////////////////////////////////////////////
//
// Command line parser helper
//
///////////////////////////////////////////////////////////////////////////////////
#define MAX_COMMAND_LINE_SIZE 2048

int isOption(int argc, char* argv[],char * paramtosearch,char * argtoparam)
{
	int param=1;
	int i,j;

	char option[MAX_COMMAND_LINE_SIZE];

	memset(option,0,sizeof(option));

	while(param<=argc)
	{
		if(argv[param])
		{
			if(argv[param][0]=='-')
			{
				memset(option,0,sizeof(option));

				j=0;
				i=1;
				while( argv[param][i] && argv[param][i]!=':' && ( j < (sizeof(option) - 1)) )
				{
					option[j]=argv[param][i];
					i++;
					j++;
				}

				if( !strcmp(option,paramtosearch) )
				{
					if(argtoparam)
					{
						argtoparam[0] = 0;

						if(argv[param][i]==':')
						{
							i++;
							j=0;
							while( argv[param][i] && j < (MAX_COMMAND_LINE_SIZE - 1) )
							{
								argtoparam[j]=argv[param][i];
								i++;
								j++;
							}
							argtoparam[j]=0;
							return 1;
						}
						else
						{
							return -1;
						}
					}
					else
					{
						return 1;
					}
				}
			}
		}
		param++;
	}

	return 0;
}

unsigned char chartoquartet(unsigned char c)
{
	if( c >= '0' && c <= '9')
	{
		return c - '0';
	}

	if( c >= 'A' && c <= 'F')
	{
		return 10 + (c - 'A');
	}

	if( c >= 'a' && c <= 'f')
	{
		return 10 + (c - 'a');
	}

	return 0;
}

int alignbyte(int cnt)
{
	if(cnt&7)
		return ((cnt>>3) + 1);
	else
		return (cnt>>3);
}

///////////////////////////////////////////////////////////////////////////////////
//
// 4B5B Encoder
//
///////////////////////////////////////////////////////////////////////////////////

#define CODE_SYNC_1  0x10
#define CODE_SYNC_2  0x11
#define CODE_RST_1   0x12
#define CODE_RST_2   0x13
#define CODE_EOP     0x14
#define CODE_SYNC_3  0x15

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

///////////////////////////////////////////////////////////////////////////////////
//
// Power delivery CRC32
//
///////////////////////////////////////////////////////////////////////////////////

#define CRC_POLY 0x04C11DB7
#define CRC_INIT 0xFFFFFFFF

uint32_t pd_crc_update(uint32_t crc, uint8_t data)
{
	uint32_t newbit, newword, rl_crc;

	for(int i=0; i<8; i++)
	{
		newbit = ((crc>>31) ^ ((data>>i)&1)) & 1;

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
		bit = (crc>>i) & 1;
		ret |= bit<<j;
	}
	return ret;
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

	for(i=0;i<4;i++)
	{
		dst_idx = encode_4b5b_quartet( frm->frame_4b5b, dst_idx, (msg->sop>>(8*(3-i)))&0xFF );
	}

	if( msg->sop == SOP || msg->sop == SOP1 || msg->sop == SOP2 || msg->sop == SOP1DEBUG || msg->sop == SOP2DEBUG )
	{
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
	}

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

	if(msg->extend)
	{
		msg->type  |= 0x200;
	}
	else
	{
		if(msg->obj_cnt)
			msg->type  |= 0x100;
	}
}

void init_ext_header(unsigned char * buf, power_delivery_msg * msg )
{
	buf[0] =  msg->data_size & 0xFF;

	buf[1] = ( (msg->data_size >> 8) & 1 )          |
		     ( ( msg->chunked & 1 ) << 7 )          |  // [15]      : Chunked
			 ( ( msg->chunk_number & 0xF ) << 3 )   |  // [14..11]  : Chunk Number
			 ( ( msg->request_chunk&1 )    << 2 );     // [10]      : Request Chunk
}

void decod_ext_header(unsigned char * buf, power_delivery_msg * msg )
{
	msg->data_size     = (buf[0] & 0xFF);                  // [8..0]    : Data Size
	msg->data_size    |= ((unsigned short)(buf[1]&1)<<8);

	msg->request_chunk = (buf[1]>>2) & 0x1;                // [10]      : Request Chunk
	msg->chunk_number  = (buf[1]>>3) & 0xF;                // [14..11]  : Chunk Number
	msg->chunked       = (buf[1]>>7) & 0x1;                // [15]      : Chunked
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

	if(!state)
	{
		dst_i++;
		dst_i++;
	}
	else
	{
		frm->bmc_frame[dst_i>>3] |= ( 0x80 >> (dst_i&7) );
		dst_i++;
		frm->bmc_frame[dst_i>>3] |= ( 0x80 >> (dst_i&7) );
		dst_i++;
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

void prepare_frame( power_delivery_msg * msg )
{
	memset( msg->frm.frame_user,0, sizeof(msg->frm.frame_user));

	msg->frm.frame_user_size = 16 + (msg->obj_cnt*4*8);

	init_header(msg->frm.frame_user, msg);
	memcpy(msg->header,msg->frm.frame_user,2);

	if( msg->extend )
		 init_ext_header((unsigned char *)&msg->objs, msg );

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

		msg->sop = 0;
		j = 0;
		while( ptr < endptr && dat != 0xFF && j < 4 )
		{
			ptr = bmcdecode(ptr,&dat);

			c = inv_conv_4b5b[dat];

			if( c >= 0x10 )
			{
				msg->sop |= ((uint32_t)(c)<<(8*(3-j)));
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

		if(msg->extend)
			decod_ext_header((unsigned char*)&msg->objs, msg );

	}
	else
	{
		if( i > (96/2) && ( frame[i] == 1 && frame[i+1] == 1 && frame[i+2] == 1 ) )
		{
			// Hard Reset or Cable Reset...
			unsigned char * ptr;
			unsigned char * endptr;
			unsigned char dat;

			dat = 0x00;

			endptr = (unsigned char *)frame + size;
			ptr = (unsigned char *)&frame[i];

			// SOP
			msg->sop = 0;
			j = 0;
			while( ptr < endptr && dat != 0xFF && j < 4 )
			{
				ptr = bmcdecode(ptr,&dat);

				c = inv_conv_4b5b[dat];

				if( c >= 0x10 )
				{
					msg->sop |= ((uint32_t)(c)<<(8*(3-j)));
				}

				j++;
			}

			msg->header[0] = 0x00;
			msg->header[1] = 0x00;
		}
		else
		{
			return -3;
		}
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

	return 1;
}

void print_msg( power_delivery_msg * msg , char * head)
{
	int i;
	unsigned char code;

	fprintf(stderr,"%s ",head);

	for(i=0;i<4;i++)
	{
		code = (msg->sop>>(8*(3-i)))&0xFF;
		switch( code )
		{
			case CODE_SYNC_1:
				fprintf(stderr,"SYNC_1 ");
			break;
			case CODE_SYNC_2:
				fprintf(stderr,"SYNC_2 ");
			break;
			case CODE_RST_1:
				fprintf(stderr,"RST_1 ");
			break;
			case CODE_RST_2:
				fprintf(stderr,"RST_2 ");
			break;
			case CODE_EOP:
				fprintf(stderr,"EOP ");
			break;
			case CODE_SYNC_3:
				fprintf(stderr,"SYNC_3 ");
			break;
			default:
				fprintf(stderr,"bad sop? (0x%x) ", code);
			break;
		}
	}

	fprintf(stderr,"[ ");

	for(i=0;i<2;i++)
	{
		fprintf(stderr,"%.2x ", msg->header[i]);
	}
	fprintf(stderr,"] ");

	fprintf(stderr,"[ ");

	for(i=0;i<msg->obj_cnt * 4;i++)
	{
		fprintf(stderr,"%.2x ", msg->objs[i]);
	}

	fprintf(stderr,"] ");

	fprintf(stderr,"%.8x ", msg->crc);

	if( msg->good_crc )
	{
		fprintf(stderr,"(Valid CRC)" );
	}
	else
	{
		fprintf(stderr,"(Bad CRC)" );
	}

	if(msg->extend)
	{
		fprintf(stderr," (ext_msg_type:%s (%d) dualrole:%d pwrrole:%d rev:%d msg_id:%d extend:%d objcnt:%d chunked:%d chunk_number:%d request_chunk:%d data_size:%d)",
		pd_extended_msg[msg->type&0x1F], msg->type&0x1F,msg->dual_role,msg->pwr_role,msg->rev,msg->id,msg->extend,msg->obj_cnt,msg->chunked,msg->chunk_number,msg->request_chunk,msg->data_size);
	}
	else
	{
		if(msg->obj_cnt)
		{
			fprintf(stderr," (data_msg_type:%s (%d) dualrole:%d pwrrole:%d rev:%d msg_id:%d extend:%d objcnt:%d)",pd_data_msg[msg->type&0x1F], msg->type&0x1F,msg->dual_role,msg->pwr_role,msg->rev,msg->id,msg->extend,msg->obj_cnt);
		}
		else
		{
			fprintf(stderr," (cmd_msg_type:%s (%d) dualrole:%d pwrrole:%d rev:%d msg_id:%d extend:%d objcnt:%d)",pd_ctrl_msg[msg->type&0x1F],msg->type&0x1F, msg->dual_role,msg->pwr_role,msg->rev,msg->id,msg->extend,msg->obj_cnt);
		}
	}
	fprintf(stderr,"\n");

	if( msg->type == DATA_MSG_ID_REQUEST )
	{
		uint32_t * tmpptr;

		tmpptr = (uint32_t *)&msg->objs;
		fprintf(stderr,"\nRequest:\n");

		fprintf(stderr,"Object position : %d\n",(tmpptr[0]>>28)&0x7);
		fprintf(stderr,"GiveBack flag : %d\n",(tmpptr[0]>>27)&1);
		fprintf(stderr,"Capability Mismatch : %d\n",(tmpptr[0]>>26)&1);
		fprintf(stderr,"USB Communications Capable : %d\n",(tmpptr[0]>>25)&1);
		fprintf(stderr,"No USB Suspend : %d\n",(tmpptr[0]>>24)&1);
		fprintf(stderr,"Unchunked Extended Messages Supported : %d\n",(tmpptr[0]>>23)&1);
		fprintf(stderr,"Reserved - Shall be set to zero : 0x%x\n",(tmpptr[0]>>20)&7);
		fprintf(stderr,"Operating Current in 10mA units : %d (%f A)\n",((tmpptr[0]>>10)&0x3FF), (float)((tmpptr[0]>>10)&0x3FF) * 0.01);
		fprintf(stderr,"Maximum Current in 10mA units : %d (%f A)\n",((tmpptr[0]>>0)&0x3FF), (float)((tmpptr[0]>>0)&0x3FF) * 0.01);
	}

	if( msg->type == DATA_MSG_ID_SOURCE_CAPABILITIES )
	{
		uint32_t * tmpptr;

		tmpptr = (uint32_t *)&msg->objs;
		fprintf(stderr,"\nSource Capabilites:\n");

		for(i=0;i<msg->obj_cnt;i++)
		{
			fprintf(stderr,"------------------------------------\n");

			switch((tmpptr[i]>>30)&3)
			{
				case 0:
					fprintf(stderr,"-- Fixed supply (Vmin = Vmax) --\n");
					fprintf(stderr,"Dual-Role Power : %d\n",(tmpptr[i]>>29)&1);
					fprintf(stderr,"USB Suspend Supported : %d\n",(tmpptr[i]>>28)&1);
					fprintf(stderr,"Unconstrained Power : %d\n",(tmpptr[i]>>27)&1);
					fprintf(stderr,"USB Communications Capable : %d\n",(tmpptr[i]>>26)&1);
					fprintf(stderr,"Dual-Role Data : %d\n",(tmpptr[i]>>25)&1);
					fprintf(stderr,"Unchunked Extended Messages Supported : %d\n",(tmpptr[i]>>24)&1);
					fprintf(stderr,"Reserved : 0x%x\n",(tmpptr[i]>>22)&3);
					fprintf(stderr,"Peak Current : %d\n",(tmpptr[i]>>20)&3);
					fprintf(stderr,"Voltage in 50mV units : %d (%f V)\n",((tmpptr[i]>>10)&0x3FF), (float)((tmpptr[i]>>10)&0x3FF) * 0.05);
					fprintf(stderr,"Maximum Current in 10mA units : %d (%f A)\n",((tmpptr[i]>>0)&0x3FF), (float)((tmpptr[i]>>0)&0x3FF) * 0.01);
				break;
				case 1:
					fprintf(stderr,"-- Battery-- \n");
					fprintf(stderr,"Maximum Voltage in 50mV units : %d (%f V)\n",((tmpptr[i]>>20)&0x3FF), (float)((tmpptr[i]>>20)&0x3FF) * 0.05);
					fprintf(stderr,"Voltage in 50mV units : %d (%f V)\n",((tmpptr[i]>>10)&0x3FF), (float)((tmpptr[i]>>10)&0x3F) * 0.05);
					fprintf(stderr,"Maximum Allowable Power in 250mW units : %d (%f W)\n",((tmpptr[i]>>0)&0x3FF), (float)((tmpptr[i]>>0)&0x3FF) * 0.25);
				break;
				case 2:
					fprintf(stderr,"Variable Supply (non-Battery)\n");
					fprintf(stderr,"Maximum Voltage in 50mV units : %d (%f V)\n",((tmpptr[i]>>20)&0x3FF), (float)((tmpptr[i]>>20)&0x3FF) * 0.05);
					fprintf(stderr,"Voltage in 50mV units : %d (%f V)\n",((tmpptr[i]>>10)&0x3FF), (float)((tmpptr[i]>>10)&0x3FF) * 0.05);
					fprintf(stderr,"Maximum Current in 10mA units : %d (%f A)\n",((tmpptr[i]>>0)&0x3FF), (float)((tmpptr[i]>>0)&0x3FF) * 0.01);
				break;
				case 3:
					fprintf(stderr,"Augmented Power Data Object (APDO)\n");
				break;
			}
		}
	}

	if( msg->type == DATA_MSG_ID_SINK_CAPABILITIES )
	{
		uint32_t * tmpptr;

		tmpptr = (uint32_t *)&msg->objs;
		fprintf(stderr,"\nSink Capabilites:\n");

		for(i=0;i<msg->obj_cnt;i++)
		{
			fprintf(stderr,"------------------------------------\n");

			switch((tmpptr[i]>>30)&3)
			{
				case 0:
					fprintf(stderr,"-- Fixed supply (Vmin = Vmax) --\n");
					fprintf(stderr,"Dual-Role Power : %d\n",(tmpptr[i]>>29)&1);
					fprintf(stderr,"Higher Capability : %d\n",(tmpptr[i]>>28)&1);
					fprintf(stderr,"Unconstrained Power : %d\n",(tmpptr[i]>>27)&1);
					fprintf(stderr,"USB Communications Capable : %d\n",(tmpptr[i]>>26)&1);
					fprintf(stderr,"Dual-Role Data : %d\n",(tmpptr[i]>>25)&1);
					fprintf(stderr,"Fast Role Swap required USB Type-C Current : %d\n",(tmpptr[i]>>23)&3);
					fprintf(stderr,"Reserved : 0x%x\n",(tmpptr[i]>>20)&7);
					fprintf(stderr,"Voltage in 50mV units : %d (%f V)\n",((tmpptr[i]>>10)&0x3FF), (float)((tmpptr[i]>>10)&0x3FF) * 0.05);
					fprintf(stderr,"Operational Current in 10mA units : %d (%f A)\n",((tmpptr[i]>>0)&0x3FF), (float)((tmpptr[i]>>0)&0x3FF) * 0.01);
				break;
				case 1:
					fprintf(stderr,"-- Battery-- \n");
					fprintf(stderr,"Maximum Voltage in 50mV units : %d (%f V)\n",((tmpptr[i]>>20)&0x3FF), (float)((tmpptr[i]>>20)&0x3FF) * 0.05);
					fprintf(stderr,"Minimum Voltage in 50mV units : %d (%f V)\n",((tmpptr[i]>>10)&0x3FF), (float)((tmpptr[i]>>10)&0x3F) * 0.05);
					fprintf(stderr,"Operationnal Power in 250mW units : %d (%f W)\n",((tmpptr[i]>>0)&0x3FF), (float)((tmpptr[i]>>0)&0x3FF) * 0.25);
				break;
				case 2:
					fprintf(stderr,"Variable Supply (non-Battery)\n");
					fprintf(stderr,"Maximum Voltage in 50mV units : %d (%f V)\n",((tmpptr[i]>>20)&0x3FF), (float)((tmpptr[i]>>20)&0x3FF) * 0.05);
					fprintf(stderr,"Minimum Voltage in 50mV units : %d (%f V)\n",((tmpptr[i]>>10)&0x3FF), (float)((tmpptr[i]>>10)&0x3FF) * 0.05);
					fprintf(stderr,"Operational Current in 10mA units : %d (%f A)\n",((tmpptr[i]>>0)&0x3FF), (float)((tmpptr[i]>>0)&0x3FF) * 0.01);
				break;
				case 3:
					fprintf(stderr,"Augmented Power Data Object (APDO)\n");
				break;
			}
		}
	}

	#if 0
	for(int i=0;i<msg->frm.bmc_frame_size;i++)
	{
		if( msg->frm.bmc_frame[i>>3] & ( 0x80 >> (i&7) ) )
		{
			fprintf(stderr,"1");
		}
		else
		{
			fprintf(stderr,"0");
		}
	}
	printf("\n");
	#endif
}

int print_byte( power_delivery_msg * msg )
{
	int i;

	printf("\n");

	for(i=0;i<alignbyte(msg->frm.bmc_frame_size);i++)
	{
		if(!(i&0xF))
			printf("\n");

		printf("0x%.2X,", msg->frm.bmc_frame[i]);

	}

	printf("\n");

	return alignbyte(msg->frm.bmc_frame_size);
}

unsigned char hex2quartet(unsigned char c)
{
	if(c >= 'A' && c <= 'F')
		return 10 + (c-'A');

	if(c >= '0' && c <= '9')
		return (c-'0');

	if(c >= 'a' && c <= 'f')
		return 10 + (c-'a');

	return 0;
}

int send_msg(int serial,  power_delivery_msg * msg)
{
	char tmpstr[8192];
	int i,ret;

	sprintf(tmpstr,"SEND");

	for(i=0;i<msg->frm.bmc_frame_size/8;i++)
	{
		sprintf(&tmpstr[4 + (i*2)],"%.2X",msg->frm.bmc_frame[i]);
	}

	strcat(tmpstr,"\n");

	ret = write_serial_str(serial,(char*)&tmpstr);

	print_msg( msg, "Sent :");

	return ret;
}

void init_message( app_ctx * ctx, power_delivery_msg * msg, int type)
{
	memset(msg,0,sizeof(power_delivery_msg));
	msg->sop       = SOP;
	msg->type      = type;
	msg->dual_role = ctx->dual_role;
	msg->rev       = ctx->protocolrev;

	msg->pwr_role  = ctx->pwr_role;
	msg->id        = ctx->id_cnt;
	msg->obj_cnt   = 0;
	msg->extend    = 0;
}

int wait_msg( app_ctx * ctx, power_delivery_msg * msg, int timeout)
{
	int i,j;
	char serialin[1024];
	unsigned char rx_ser_frame[1024];
	unsigned char decod_frame[1024*8];
	int size;
	int state;
	int statecnt;
	int decod_frame_size;
	int ret;

	ret = read_serial_str(ctx->serial, (char*)&serialin, sizeof(serialin), timeout);
	if( ret >= 0 )
	{
		//printf("!! %s\n",serialin);
		if( !strncmp(serialin,"RECV",4) )
		{
			i = 0;
			j = 0;
			statecnt = 0;

			while(serialin[4+i] && (i<sizeof(rx_ser_frame)*2))
			{
				rx_ser_frame[(i>>1)] <<= 4;
				rx_ser_frame[(i>>1)] |= (hex2quartet(serialin[4+i]) & 0xF);
				i++;
			}
			size = i/2;

			if(rx_ser_frame[0] & 0x80 )
				state = 1;
			else
				state = 0;

			i = 0;
			j = 0;
			memset(decod_frame,0,sizeof(decod_frame));
			while( i < size * 8 )
			{
				if( rx_ser_frame[i>>3] & (0x80>>(i&7)) )
				{
					if(state)
						statecnt++;
					else
					{
						decod_frame[j++] = statecnt;
						statecnt = 1;
						state ^= 1;
					}
				}
				else
				{
					if(!state)
						statecnt++;
					else
					{
						decod_frame[j++] = statecnt;
						statecnt = 1;
						state ^= 1;
					}
				}
				i++;
			}

			decod_frame_size = j;

			ret = rcv_frame( msg, decod_frame, decod_frame_size );

			if(ret>=0)
				print_msg( msg, "Received :");

		}
		else
		{
			ret = -4;
		}
	}

	return ret;
}

int process_sink_caps(app_ctx * app, power_delivery_msg * sink_caps_msg2 )
{
	power_delivery_msg sink_caps_msg;

	memset(&sink_caps_msg,0,sizeof(power_delivery_msg));
	sink_caps_msg.sop       = SOP;
	sink_caps_msg.type      = CTRL_MSG_ID_REJECT;//DATA_MSG_ID_SOURCE_CAPABILITIES;
	sink_caps_msg.dual_role = app->dual_role;
	sink_caps_msg.rev       = app->protocolrev;

	sink_caps_msg.pwr_role  = app->pwr_role;
	sink_caps_msg.id        = app->id_cnt;//soft_reset_msg->id;
	sink_caps_msg.obj_cnt   = 0;//3;
	sink_caps_msg.extend    = 0;

	uint32_t * tmpptr;
	tmpptr = (uint32_t *)&sink_caps_msg.objs;

	*tmpptr++ = 0x0801912C;
	*tmpptr++ = 0x0002d0de;
	*tmpptr++ = 0x0003c0a7;

	prepare_frame(&sink_caps_msg);
	send_msg(app->serial,  &sink_caps_msg);

	app->id_cnt = 0;

	return 0;
}

int process_getstatus(app_ctx * app)
{
	power_delivery_msg sink_caps_msg;

	memset(&sink_caps_msg,0,sizeof(power_delivery_msg));
	sink_caps_msg.sop       = SOP;
	sink_caps_msg.type      = CTRL_MSG_ID_Get_Status;
	sink_caps_msg.dual_role = app->dual_role;
	sink_caps_msg.rev       = app->protocolrev;

	sink_caps_msg.pwr_role  = app->pwr_role;
	sink_caps_msg.id        = app->id_cnt;
	sink_caps_msg.obj_cnt   = 0;
	sink_caps_msg.extend    = 0;

	prepare_frame(&sink_caps_msg);
	send_msg(app->serial,  &sink_caps_msg);

	return 0;
}

int process_source_caps(app_ctx * app, power_delivery_msg * src_caps_msg )
{
	int i;
	int cap_select;
	uint32_t * tmpptr;
	power_delivery_msg req_msg;


	cap_select = -1;
	tmpptr = (uint32_t *)&src_caps_msg->objs;

	for(i=0;i<src_caps_msg->obj_cnt;i++)
	{
		switch((tmpptr[i]>>30)&3)
		{
			case 0:
				if( ((tmpptr[i]>>10)&0x3FF) == 100)
					cap_select = i;
			break;
			case 1:
				if( ((tmpptr[i]>>10)&0x3FF) == 100)
					cap_select = i;
			break;
			case 2:
				if( ((tmpptr[i]>>10)&0x3FF) == 100)
					cap_select = i;
			break;
			case 3:
			break;
		}
	}

//cap_select = 1;
	if(cap_select>= 0)
	{
		memset(&req_msg,0,sizeof(power_delivery_msg));
		req_msg.sop       = SOP;
		req_msg.type      = DATA_MSG_ID_REQUEST; // Request
		req_msg.dual_role = app->dual_role;
		req_msg.rev       = app->protocolrev;

		req_msg.pwr_role  = app->pwr_role;
		req_msg.id        = app->id_cnt;
		req_msg.obj_cnt   = 1;
		req_msg.extend    = 0;

		uint32_t * tmpptr;
		tmpptr = (uint32_t *)&req_msg.objs;

		*tmpptr = ((cap_select+1)<<28) | (500/10) | ((500/10) << 10);
		prepare_frame(&req_msg);
		send_msg(app->serial,  &req_msg);
		//app->id_cnt = (app->id_cnt + 1)&7;
		return 1;
	}

	return -4;
}

int send_source_caps(app_ctx * app)
{
	power_delivery_msg req_msg;

	memset(&req_msg,0,sizeof(power_delivery_msg));
	req_msg.sop       = SOP;
	req_msg.type      = DATA_MSG_ID_SOURCE_CAPABILITIES;
	req_msg.dual_role = app->dual_role;
	req_msg.rev       = app->protocolrev;

	req_msg.pwr_role  = app->pwr_role;
	req_msg.id        = app->id_cnt;
	req_msg.obj_cnt   = 1;
	req_msg.extend    = 0;

	uint32_t * tmpptr;
	tmpptr = (uint32_t *)&req_msg.objs;

//	*tmpptr++ =  0x2801900A;//((5000/50)<<10) | ((100/10)<<0);
	*tmpptr++ =  ((5000/50)<<10) | ((3000/10)<<0);

//    *tmpptr++ = ((5000/50)<<10) | ((1000/10)<<0);// | (1<<25);


//	*tmpptr = ((12000/50)<<10) | ((200/10)<<0);
	prepare_frame(&req_msg);
	send_msg(app->serial,  &req_msg);

	return 1;
}

int send_and_wait_ack(app_ctx * ctx, power_delivery_msg * msg)
{
	int l,ret;
	power_delivery_msg gcrcmsg;

	prepare_frame(msg);

	l = 0;
	memset(&gcrcmsg,0,sizeof(power_delivery_msg));
	while ( (gcrcmsg.type != CTRL_MSG_ID_GOODCRC) && l<3)
	{
		send_msg(ctx->serial,  msg);

		memset(&gcrcmsg,0,sizeof(power_delivery_msg));
		ret = wait_msg( ctx, &gcrcmsg, 200);
		l++;
	};

	if( ret >= 0 && gcrcmsg.type == CTRL_MSG_ID_GOODCRC )
	{
		if(gcrcmsg.id == msg->id)
		{
			ctx->id_cnt = (ctx->id_cnt + 1)&7;

			return 1;
		}
		else
		{
			printf("Good CRC : Bad ID counter !\n");
		}
	}

	return 0;
}

int test_usb_cable(app_ctx * ctx)
{
	int ret,i;
	power_delivery_msg msg;
	power_delivery_msg sndmsg;

	init_message( ctx, &sndmsg, CTRL_MSG_ID_RSVD_00);
	sndmsg.sop = HARDRESET;
	prepare_frame(&sndmsg);
	send_msg(ctx->serial,  &sndmsg);

	i = 0;
	for(;;)
	{
		memset(&msg,0,sizeof(power_delivery_msg));
		ret = wait_msg( ctx, &msg, 150);

		if( ret == ERR_TIMEOUT )
		{
			init_message( ctx, &sndmsg, CTRL_MSG_ID_Soft_Reset);
			if(i&1)
				sndmsg.sop = SOP1;
			else
				sndmsg.sop = SOP2;

			prepare_frame(&sndmsg);
			send_msg(ctx->serial,  &sndmsg);
			i++;
		}
		else
		{
			if(msg.type == CTRL_MSG_ID_GOODCRC)
			{
			//	init_message( ctx, &sndmsg, EXTENDED_MSG_ID_GET_MANUFACTURER_INFO);
				sndmsg.type = EXTENDED_MSG_ID_GET_MANUFACTURER_INFO;
				sndmsg.extend = 1;
				sndmsg.data_size = 2;
				sndmsg.objs[2] = 0x00;
				sndmsg.objs[3] = 0x00;

				sndmsg.obj_cnt = 1;
				send_and_wait_ack( ctx, &sndmsg);

				memset(&msg,0,sizeof(power_delivery_msg));
				ret = wait_msg( ctx, &msg, 3000);

				memset(&msg,0,sizeof(power_delivery_msg));
				ret = wait_msg( ctx, &msg, 3000);
			}
		}
	}

}

int send_source_caps_init(app_ctx * ctx)
{
	int ret;
	power_delivery_msg msg;
	power_delivery_msg sndmsg;

/*
	do
	{
		memset(&msg,0,sizeof(power_delivery_msg));
		ret = wait_msg( ctx, &msg, 100);
	}while( msg.sop != HARDRESET );
*/
	init_message( ctx, &sndmsg, CTRL_MSG_ID_RSVD_00);
	sndmsg.sop = SOP1;
	prepare_frame(&sndmsg);
	send_msg(ctx->serial,  &sndmsg);

	do
	{
		memset(&msg,0,sizeof(power_delivery_msg));
		ret = wait_msg( ctx, &msg, 150);

		if( ret == ERR_TIMEOUT )
		{
			//send_source_caps( ctx );
		}

	}while ( msg.type != CTRL_MSG_ID_GOODCRC );

	ctx->id_cnt = (ctx->id_cnt + 1)&7;

	// Wait the source caps or request message
	memset(&msg,0,sizeof(power_delivery_msg));
	ret = wait_msg( ctx, &msg, 3000);

	if(ret >= 0)
	{
		if(msg.type == DATA_MSG_ID_REQUEST)
		{
			init_message( ctx, &sndmsg, CTRL_MSG_ID_ACCEPT);
			send_and_wait_ack( ctx, &sndmsg);

			usleep(1000*50);

			init_message( ctx, &sndmsg, CTRL_MSG_ID_PS_RDY);
			send_and_wait_ack( ctx, &sndmsg);

			init_message( ctx, &sndmsg, CTRL_MSG_ID_Get_Revision);
			send_and_wait_ack( ctx, &sndmsg);

			memset(&msg,0,sizeof(power_delivery_msg));
			ret = wait_msg( ctx, &msg, 3000);

/*
			init_message( ctx, &sndmsg, CTRL_MSG_ID_Get_Sink_Cap);
			send_and_wait_ack( ctx, &sndmsg);
			memset(&msg,0,sizeof(power_delivery_msg));
			ret = wait_msg( ctx, &msg, 3000);

	//		do{
			memset(&msg,0,sizeof(power_delivery_msg));
			ret = wait_msg( ctx, &msg, 3000);

//			}while(1);

			memset(&msg,0,sizeof(power_delivery_msg));
			ret = wait_msg( ctx, &msg, 3000);

			init_message( ctx, &sndmsg, CTRL_MSG_ID_Get_Country_Codes);
			send_and_wait_ack( ctx, &sndmsg);

			memset(&msg,0,sizeof(power_delivery_msg));
			ret = wait_msg( ctx, &msg, 3000);

			init_message( ctx, &sndmsg, CTRL_MSG_ID_Get_Status);
			send_and_wait_ack( ctx, &sndmsg);

			memset(&msg,0,sizeof(power_delivery_msg));
			ret = wait_msg( ctx, &msg, 3000);

			init_message( ctx, &sndmsg, EXTENDED_MSG_ID_GET_BATTERY_CAP);
			sndmsg.extend = 1;
			sndmsg.data_size = 1;
			sndmsg.objs[2] = 0x00;
			sndmsg.obj_cnt = 1;
			send_and_wait_ack( ctx, &sndmsg);

			memset(&msg,0,sizeof(power_delivery_msg));
			ret = wait_msg( ctx, &msg, 3000);
*/
			do
			{
				init_message( ctx, &sndmsg, EXTENDED_MSG_ID_GET_MANUFACTURER_INFO);
				sndmsg.extend = 1;
				sndmsg.data_size = 2;
				sndmsg.objs[2] = 0x01;
				sndmsg.objs[3] = 0x00;

				sndmsg.obj_cnt = 1;
				send_and_wait_ack( ctx, &sndmsg);

				memset(&msg,0,sizeof(power_delivery_msg));
				ret = wait_msg( ctx, &msg, 3000);
			}while(1);

		}
	}

	return ret;
}

int process_soft_reset(app_ctx * app, power_delivery_msg * soft_reset_msg )
{
	power_delivery_msg accept_msg;

	memset(&accept_msg,0,sizeof(power_delivery_msg));
	accept_msg.sop       = SOP;
	accept_msg.type      = CTRL_MSG_ID_ACCEPT;
	accept_msg.dual_role = app->dual_role;
	accept_msg.rev       = app->protocolrev;

	accept_msg.pwr_role  = app->pwr_role;
	accept_msg.id        = app->id_cnt;
	accept_msg.obj_cnt   = 0;
	accept_msg.extend    = 0;

	prepare_frame(&accept_msg);
	send_msg(app->serial,  &accept_msg);

	app->id_cnt = 0;

	return 1;
}

int send_hardreset(app_ctx * app)
{
	power_delivery_msg msg;

	memset(&msg,0,sizeof(power_delivery_msg));
	msg.sop       = HARDRESET;

	prepare_frame(&msg);
	send_msg(app->serial,  &msg);

	return 1;
}

int main (int argc, char ** argv)
{
	int ret,s;
	char tmp_str[MAX_COMMAND_LINE_SIZE];
	unsigned char decod_frame[1024*8];
	int decod_frame_size;
	int encode;
	app_ctx ctx;

	memset(&ctx,0,sizeof(app_ctx));

	power_delivery_msg msg;
	power_delivery_msg rcv_msg;

	memset(&msg,0,sizeof(power_delivery_msg));

	encode = 1;
	decod_frame_size = 0;

	fprintf(stderr,"Power delivery frame bitstream generator v0.1 (-?/-help for the command line options)\n\n");

	if( isOption( argc, argv, "help", NULL) == 1 )
	{
		printhelp();
		exit(1);
	}

	if( isOption( argc, argv, "?", NULL) == 1 )
	{
		printhelp();
		exit(1);
	}

	msg.sop = SOP;

	if( isOption( argc, argv,"msg_type",(char*)&tmp_str) == 1 )
	{
		msg.type = (int)atoi(tmp_str);
	}

	if( isOption( argc, argv,"dualrole",(char*)&tmp_str) == 1 )
	{
		ctx.dual_role = (int)atoi(tmp_str);
		msg.dual_role = ctx.dual_role;
	}

	if( isOption( argc, argv,"pwrrole",(char*)&tmp_str) == 1 )
	{
		ctx.pwr_role = (int)atoi(tmp_str);
		msg.pwr_role = ctx.pwr_role;
	}

	ctx.protocolrev = 2;
	if( isOption( argc, argv,"rev",(char*)&tmp_str) == 1 )
	{
		ctx.protocolrev = (int)atoi(tmp_str);
		msg.rev = ctx.protocolrev;
	}

	ctx.id_cnt = 0;
	msg.id = ctx.id_cnt;
	if( isOption( argc, argv,"msg_id",(char*)&tmp_str) == 1 )
	{
		ctx.id_cnt = (int)atoi(tmp_str);
		msg.id = ctx.id_cnt;
	}

	msg.extend = 0;
	if( isOption( argc, argv,"extend",(char*)&tmp_str) == 1 )
	{
		msg.extend = (int)atoi(tmp_str);
	}

	msg.obj_cnt = 0;
	s = 0;
	tmp_str[0] = 0;
	if( isOption( argc, argv,"objs",(char*)&tmp_str) == 1 )
	{
		int i = 0;
		int j = 0;
		unsigned char * objs_ptr;
		objs_ptr = (unsigned char *)&msg.objs;

		memset(objs_ptr,0,sizeof(msg.objs));
		while(tmp_str[i] && j < (8*7) )
		{
			objs_ptr[j>>1] = (objs_ptr[j>>1]<<4) | chartoquartet(tmp_str[i]);
			i++;
			j++;
		}

		s += (j*4);
		if ( s & 0x1F)
			s = (s & ~0x1F) + 0x20;

		msg.obj_cnt = s / 32;
	}

	tmp_str[0] = 0;
	if( isOption( argc, argv,"decode",(char*)&tmp_str) == 1 )
	{
		int state = 0;
		int statecnt = 0;

		int i = 0;
		int j = 0;
		memset(decod_frame,0,sizeof(decod_frame));
		while( tmp_str[i] )
		{
			if( tmp_str[i] == '0' || tmp_str[i] == '1' )
			{
				if( (tmp_str[i] - '0') == state)
				{
				   statecnt++;
				}
				else
				{
					if(statecnt)
						decod_frame[j++] = statecnt;
					statecnt = 1;
					state ^= 1;
				}
			}
			i++;
		}

		decod_frame_size = j;

		/*for(i=0;i<decod_frame_size;i++)
		{
			printf("%d",decod_frame[i]);
		}
		printf("\n");*/

		encode = 0;
	}

	if( isOption( argc, argv,"gengcrc",NULL) == 1 )
	{
		// rev : 2 bits, dualrole 1, pwrrole : 1, idcnt: 3 = 7bits

		// [6..5][4][3][2..0]
		unsigned char frame;
		int offset;

		offset = 0;
		msg.type = CTRL_MSG_ID_GOODCRC;
		for(frame=0;frame<128;frame++)
		{
			ctx.protocolrev = (frame >> 5) & 3;
			msg.rev = (frame >> 5) & 3;
			ctx.pwr_role = (frame >> 3) & 1;
			msg.pwr_role = (frame >> 3) & 1;
			ctx.dual_role = (frame >> 4) & 1;
			msg.dual_role = (frame >> 4) & 1;
			ctx.id_cnt = frame&0x7;
			msg.id = frame&0x7;

			if(!(frame&0x7))
				fprintf(stdout,"\n// ----------------------- 0x%.8X -----------------------\n",offset);

			fprintf(stdout,"\n// msg_type:%d dualrole:%d pwrrole:%d rev:%d msg_id:%d extend:%d objcnt:%d (",msg.type,msg.dual_role,msg.pwr_role,msg.rev,msg.id,msg.extend,msg.obj_cnt);
			for(int i=0;i<(msg.obj_cnt * 4);i++)
			{
				fprintf(stdout,"%.2x",*(((unsigned char*)(msg.objs)) + i) );
			}
			fprintf(stdout,")");

			prepare_frame( &msg );

			offset += print_byte( &msg );
		}

		exit(1);
	}

	tmp_str[0] = 0;
	if( isOption( argc, argv,"ser",(char*)&tmp_str) == 1 )
	{
		char serialin[512];
		ctx.serial = open_and_cfg_serial((char*)&tmp_str);
		if(ctx.serial >= 0)
		{
			// Sync
			write_serial_str(ctx.serial, "\n");
			write_serial_str(ctx.serial, "\n");
			write_serial_str(ctx.serial, "\n");
			write_serial_str(ctx.serial, "RXOF\n");
			write_serial_str(ctx.serial, "RXOF\n");
			write_serial_str(ctx.serial, "RXOF\n");
			write_serial_str(ctx.serial, "\n");
			// flush
			if( read_serial_str(ctx.serial, (char*)&serialin, sizeof(serialin),100) < 0 )
			{
			}

			for(int i=0;i<64;i++)
			{
				write_serial_str(ctx.serial, "PING\n");
				serialin[0] = 0;
				if( read_serial_str(ctx.serial, (char*)&serialin, sizeof(serialin),1000) < 0 )
				{
					fprintf(stderr,"Read error !\n\n");
					close_serial(ctx.serial);
					exit(-1);
				}

				if( strcmp( serialin, "PONG") )
				{
					fprintf(stderr,"Bad response !\n\n");
					close_serial(ctx.serial);
					exit(-1);
				}
			}

			printf("Serial link ok\n");

			char tmpstr_serfrm[16];
			sprintf(tmpstr_serfrm,"CFGM%X\n",( ((ctx.protocolrev&3)<<2) | ((ctx.dual_role&1)<<1) | (ctx.pwr_role & 1)));
			write_serial_str(ctx.serial, tmpstr_serfrm);

			if( isOption( argc, argv,"spy",NULL) == 1 )
			{
				write_serial_str(ctx.serial, "CRC0\n");
				write_serial_str(ctx.serial, "RXON\n");
				while(1)
				{
					ret = wait_msg( &ctx, &rcv_msg, 1000);
				}
			}
			else
			{
				write_serial_str(ctx.serial, "CRC1\n");
				write_serial_str(ctx.serial, "RXON\n");


				//////////////////////////////////////////////////////////////////////////////////////////////////////:
				// Source Mode
				//////////////////////////////////////////////////////////////////////////////////////////////////////:

				if( isOption( argc, argv,"source",NULL) == 1 )
				{
					send_source_caps_init(&ctx);

					close_serial(ctx.serial);
					exit(1);
				}


				//////////////////////////////////////////////////////////////////////////////////////////////////////:
				// Sink Mode
				//////////////////////////////////////////////////////////////////////////////////////////////////////:

				if( isOption( argc, argv,"sink",NULL) == 1 )
				{
					//send_hardreset(&ctx);

					do
					{
						ret = wait_msg( &ctx, &rcv_msg, 1000);
						if(ret>=0)
						{
							if(rcv_msg.type == CTRL_MSG_ID_GOODCRC)
							{
								if(rcv_msg.id == ctx.id_cnt)
								{
									ctx.id_cnt = (ctx.id_cnt + 1)&7;
								}
								else
								{
									printf("Good CRC : Bad ID !!!!\n");
								}
							}

							// Source capabilities ?
							if( rcv_msg.type == CTRL_MSG_ID_Get_Sink_Cap )
							{
								process_sink_caps(&ctx, &rcv_msg );
							}

							// Source capabilities ?
							if( rcv_msg.type == DATA_MSG_ID_SOURCE_CAPABILITIES )
							{
								//process_sink_caps(&ctx, &rcv_msg );
								process_source_caps( &ctx, &rcv_msg );
							}

							// Soft Reset ?
							if( rcv_msg.type == CTRL_MSG_ID_Soft_Reset )
							{
								process_soft_reset( &ctx, &rcv_msg );
							}
						}


					}while(1);

					close_serial(ctx.serial);
					exit(1);
				}

				if( isOption( argc, argv,"test_cable",NULL) == 1 )
				{
					test_usb_cable( &ctx );
				}
			}

			close_serial(ctx.serial);
			exit(1);

			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		else
		{
			fprintf(stderr,"Failed to open %s !\n\n",tmp_str);
		}
	}

	if( encode )
	{
		fprintf(stderr,"msg_type:%d dualrole:%d pwrrole:%d rev:%d msg_id:%d extend:%d objcnt:%d (",msg.type,msg.dual_role,msg.pwr_role,msg.rev,msg.id,msg.extend,msg.obj_cnt);
		for(int i=0;i<(msg.obj_cnt * 4);i++)
		{
			fprintf(stderr,"%.2x",*(((unsigned char*)(msg.objs)) + i) );
		}
		fprintf(stderr,")\n");

		prepare_frame( &msg );
		print_msg( &msg ,"");

		print_byte( &msg );

	}
	else
	{
		// Decode
		rcv_frame( &rcv_msg , decod_frame, decod_frame_size );
		print_msg( &rcv_msg ,"");
	}
	ret = 0;

	exit(ret);
}

