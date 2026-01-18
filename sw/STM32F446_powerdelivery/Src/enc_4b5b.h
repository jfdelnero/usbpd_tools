///////////////////////////////////////////////////////////////////////////////
// File : enc_4b5b.h
// Contains: USB Power delivery 4b5b encoder / decoder
//
// Written by: Jean-François DEL NERO
//
// Change History (most recent first):
///////////////////////////////////////////////////////////////////////////////

#define CODE_SYNC_1  0x10
#define CODE_SYNC_2  0x11
#define CODE_RST_1   0x12
#define CODE_RST_2   0x13
#define CODE_EOP     0x14
#define CODE_SYNC_3  0x15

extern const unsigned char conv_4b5b[];
extern const unsigned char inv_conv_4b5b[];

int encode_4b5b_quartet(unsigned char * dstbuf, int idx, unsigned char quartet);
