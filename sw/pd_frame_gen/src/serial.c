///////////////////////////////////////////////////////////////////////////////////
// File : serial.c
// Contains: serial in/out functions
//
// Written by: Jean-François DEL NERO
///////////////////////////////////////////////////////////////////////////////////
#ifndef ANSI_FILE
#define _GNU_SOURCE 1
#endif

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef ANSI_FILE
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#endif

#include "serial.h"

#ifdef ANSI_FILE
FILE* open_and_cfg_serial(char *devpath)
#else
int open_and_cfg_serial(char *devpath)
#endif
{

#ifdef ANSI_FILE
	FILE * port;

	port = fopen(devpath,"wb");

#else
	struct termios options;
	int port,ret;

	port = open(devpath, O_RDWR | O_NOCTTY);
	if(port == -1)
		return ERR_ACCESS;

	usleep(10000);

	tcflush(port, TCIOFLUSH);

	ret = tcgetattr(port, &options);
	if (ret)
	{
		close(port);
		return ERR_ACCESS;
	}

	//Disable these flags :
	//INLCR: Translate NL to CR on input.
	//IGNCR: Ignore carriage return on input.
	//ICRNL: Translate carriage return to newline on input (unless IGNCR is set).
	//IXON:  Enable XON/XOFF flow control on output.
	//IXOFF: Enable XON/XOFF flow control on input.
	options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);

	//Disable these flags :
	//ONLCR: (XSI) Map NL to CR-NL on output.
	//OCRNL: Map CR to NL on output.
	//ONOCR: Don't output CR at column 0.
	options.c_oflag &= ~(ONLCR | OCRNL | ONOCR);

	//Disable these flags :
	//ECHO:   Echo input characters.
	//ECHONL: If ICANON is also set, echo the NL character even if ECHO is not set.
	//ICANON: Enable canonical mode (described below).
	//ISIG:   When any of the characters INTR, QUIT, SUSP, or DSUSP are received, generate the corresponding signal.
	//IEXTEN: Enable implementation-defined input processing.
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	// 8N1
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	options.c_cc[VTIME] = 7;
	options.c_cc[VMIN] = 0;

	cfsetospeed(&options, B1000000);
	cfsetispeed(&options, cfgetospeed(&options));

	ret = tcsetattr(port, TCSANOW, &options);
	if (ret)
	{
		close(port);
		return ERR_ACCESS;
	}

#endif

	return port;
}

#ifdef ANSI_FILE
int fread_to_serial(FILE * file, unsigned char * buf, int timeout)
#else
int fread_to_serial(int file, unsigned char * buf, int timeout)
#endif
{
	fd_set rfds;
	struct timeval tv;
	int ret, fd;

#ifdef ANSI_FILE
	fd = fileno(file);
#else
	fd = file;
#endif
	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);

	tv.tv_sec = timeout/1000;
	tv.tv_usec = (timeout - ((timeout/1000)*1000))*1000;

	ret = select(fd+1, &rfds, NULL, NULL, &tv);

	if(ret == 0)
	{
		//fprintf(stderr, "Poll timed out !\n");
		return ERR_TIMEOUT;
	}
	else if (ret < 0)
	{
		return ERR_ACCESS;
	}
	else if(FD_ISSET(fd, &rfds))
	{
#ifdef ANSI_FILE
		ret = fread(buf, 1, 1, file);
#else
		ret = read(file, buf, 1);
#endif

		return ret;
	}

	return 0;
}

#ifdef ANSI_FILE
int write_serial(FILE * file, unsigned char * buf,int size)
#else
int write_serial(int file, unsigned char * buf, int size)
#endif
{
    int ret;
    
    ret = 1;
#ifdef ANSI_FILE
	fwrite(buf,size,1,file);
	fflush(file);
#else
	if ( write(file, buf, size) != size )
	{
		ret = ERR_ACCESS;
	}
#endif

    return ret;
}

#ifdef ANSI_FILE
void close_serial(FILE *f)
#else
void close_serial(int f)
#endif
{
#ifdef ANSI_FILE
		fclose(f);
#else
		close(f);
#endif
}

#ifdef ANSI_FILE
int write_serial_str(FILE * file, char * buf)
#else
int write_serial_str(int file, char * buf)
#endif
{
	int size;

	size = strlen(buf);
	return write_serial(file, (unsigned char*)buf, size);
}


#ifdef ANSI_FILE
int read_serial_str(FILE * file, char * buf, int maxsize, int timeout)
#else
int read_serial_str(int file, char * buf, int maxsize, int timeout)
#endif
{
	int size,ret;

	size = 0;
	
	while(size < maxsize)
	{
		ret = fread_to_serial(file, (unsigned char*)&buf[size], timeout);
		if( ret >= 0 )
		{
			if( buf[size] == '\n' )
			{
				buf[size] = 0;
				return size;
			}
			size++;
		}
		else
			return ret;
	}
	
	buf[size-1] = 0;
	
	return size-1;
}

