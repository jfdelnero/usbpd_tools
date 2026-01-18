// Minimal print functions
// (c) Jean-François DEL NERO
// (c) HxC2001

void printchar(unsigned char c);
void print(const unsigned char *pucBuffer);
void printdec(unsigned short c, int pad);
void printdec_long(unsigned int val, int pad);
void printhex(unsigned char c);
void printhex_short(unsigned short c);
void printhex_long(unsigned long c);
void printbuf(unsigned char * buf,int size);
void printbuflong(unsigned long * buf,int size);
void printhexquartet(unsigned char c);


void dbg_printf(const unsigned char *pucBuffer, unsigned int arg1,unsigned int arg2,unsigned int arg3);

unsigned char * str_printhex_long(unsigned char * buf, unsigned long c,int nb_bytes);
unsigned char str_hex_to_byte(unsigned char * buf);

unsigned char hex2quartet(unsigned char c);
