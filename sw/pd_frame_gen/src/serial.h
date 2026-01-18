///////////////////////////////////////////////////////////////////////////////////
// File : serial.h
// Contains: serial in/out functions
//
// Written by: Jean-François DEL NERO
///////////////////////////////////////////////////////////////////////////////////

#ifdef ANSI_FILE
FILE* open_and_cfg_serial(char *devpath);
#else
int open_and_cfg_serial(char *devpath);
#endif

#ifdef ANSI_FILE
int fread_to_serial(FILE * file, unsigned char * buf, int timeout);
#else
int fread_to_serial(int file, unsigned char * buf, int timeout);
#endif

#ifdef ANSI_FILE
int write_serial(FILE * file, unsigned char * buf,int size);
#else
int write_serial(int file, unsigned char * buf, int size);
#endif

#ifdef ANSI_FILE
void close_serial(FILE *f);
#else
void close_serial(int f);
#endif

#ifdef ANSI_FILE
int read_serial_str(FILE * file, char * buf, int maxsize, int timeout);
#else
int read_serial_str(int file, char * buf, int maxsize, int timeout);
#endif

#ifdef ANSI_FILE
int write_serial_str(FILE * file, char * buf);
#else
int write_serial_str(int file, char * buf);
#endif

#define ERR_TIMEOUT -1
#define ERR_ACCESS -2
