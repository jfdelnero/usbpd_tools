// Minimal print functions
// (c) Jean-François DEL NERO
// (c) HxC2001

#include "stm32f4xx_hal.h"

#include "buildconf.h"



void printchar(unsigned char c)
{
	while(!(USART2->SR & USART_SR_TXE));
	USART2->DR = c;
}

void printhex(unsigned char c)
{
	unsigned char c1;

	c1=c>>4;

	if(c1<10)
	{
		printchar('0'+c1);
	}
	else
	{
		printchar('A'+(c1-10));
	}

	c1=c&0xF;

	if(c1<10)
	{
		printchar('0'+c1);
	}
	else
	{
		printchar('A'+(c1-10));
	}
}

void printhex_short(unsigned short c)
{
	printhex((c>>8)&0xFF);
	printhex(c&0xFF);
}

void printhex_long(unsigned long c)
{
	printhex((c>>24)&0xFF);
	printhex((c>>16)&0xFF);
	printhex((c>>8)&0xFF);
	printhex(c&0xFF);
}

void printhexquartet(unsigned char c)
{
	c = c & 0xF;

	if(c<10)
	{
		printchar('0'+c);
	}
	else
	{
		printchar('A'+(c-10));
	}
}

void print(const unsigned char *pucBuffer)
{
	int i;
	//
	// Loop while there are more characters to send.
	//
	if(!pucBuffer)
	{
		print((unsigned char*)"<NULL STR>");
		return;
	}
	i=0;
	while(pucBuffer[i])
	{
		printchar(pucBuffer[i]);
		i++;
	}
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


void dbg_printf(const unsigned char *pucBuffer, unsigned int arg1,unsigned int arg2,unsigned int arg3)
{
	int i,argpos;
	unsigned int arglist[4];

	arglist[0] = arg1;
	arglist[1] = arg2;
	arglist[2] = arg3;
	arglist[3] = 0;

	//
	// Loop while there are more characters to send.
	//
	if(!pucBuffer)
	{
		print((unsigned char*)"<NULL STR>");
		return;
	}

	argpos = 0;

	i=0;
	while(pucBuffer[i])
	{
		if(pucBuffer[i] == '%')
		{
			switch(pucBuffer[i+1])
			{
				case 'X':
					printhex_long(arglist[argpos]);
					argpos = (argpos + 1)&3;
					i = i + 2;
				break;
				case 's':
					print((unsigned char*)arglist[argpos]);
					argpos = (argpos + 1)&3;
					i = i + 2;
				break;
				default:
					printchar(pucBuffer[i]);
					i++;
				break;
			}
		}
		else
		{
			printchar(pucBuffer[i]);
			i++;
		}
	}
}

void printdec(unsigned short c, int pad)
{
	unsigned char c1,c2,c3;

	c1=c/100;
	if(c1 || pad)
	{
		pad = 1;
		printchar('0'+c1);
	}

	c2=(c/10) - (c1*10);
	if(c2 || pad)
	{
		pad = 1;
		printchar('0'+c2);
	}

	c3= c - ( (c1*100) + (c2*10) );
	printchar('0'+c3);
}

void printdec_long(unsigned int val, int pad)
{
	int div;
	unsigned char car;

	div = 1000*1000*1000;

	do
	{
		car = (val / div);
		if( car )
			pad = 1;

		if(pad)
			printchar('0'+car);

		val = val - (div*car);
		div = div / 10;
	}while(div != 0);
}

void printbuf(unsigned char * buf,int size)
{
	int i;
	print((unsigned char*)"\r\n");
	for(i=0;i<size;i++)
	{
		if(!(i&0xF))
			print((unsigned char*)"\r\n");

		printhex(buf[i]);
	}
	print((unsigned char*)"\r\n");
}

void printbuflong(unsigned long * buf,int size)
{
	int i;
	print((unsigned char*)"\r\n");
	for(i=0;i<size;i++)
	{
		if(!(i&0x3))
			print((unsigned char*)"\r\n");

		printhex_long(buf[i]);
		print((unsigned char*)" ");
	}
	print((unsigned char*)"\r\n");
}

unsigned char * str_printhex(unsigned char * buf,unsigned char c)
{
	unsigned char c1;

	c1=c>>4;

	if(c1<10)
	{
		*buf++ = '0'+c1;
	}
	else
	{
		*buf++ = 'A'+(c1-10);
	}

	c1=c&0xF;

	if(c1<10)
	{
		*buf++ = '0'+c1;
	}
	else
	{
		*buf++ = 'A'+(c1-10);
	}

	return buf;
}

unsigned char * str_printhex_long(unsigned char * buf, unsigned long c,int nb_bytes)
{
	do
	{
		nb_bytes--;

		buf = str_printhex(buf, (c>>(8*nb_bytes)) & 0xFF );

	}while(nb_bytes);

	return buf;
}

unsigned char str_hex_to_byte(unsigned char * buf)
{
	unsigned char c,i;

	c = 0x00;

	for(i=0;i<2;i++)
	{
		c = c << 4 ;

		if( *buf >= '0' && *buf <= '9' )
		{
			c |= (*buf - '0');
		}
		else
		{
			if( *buf >= 'A' && *buf <= 'F' )
			{
				c |= (*buf - 'A');
			}
		}

		buf++;
	}

	return c;
}
