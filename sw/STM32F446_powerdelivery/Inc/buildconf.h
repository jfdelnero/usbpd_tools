#ifndef CONFIG_FILE_INCLUDED
#define CONFIG_FILE_INCLUDED 1

#define CONFIG_SPI_PORT 3

// SPI Clock Divisor
// 0 -> /2  (38.4MHz / 2 -> 19.2MHz delta sigma DAC)
// 1 -> /4  (38.4MHz / 4 -> 9.6MHz delta sigma DAC)
// 2 -> /8  (38.4MHz / 8 ->  4.8MHz delta sigma DAC)
// 3 -> /16 (38.4MHz / 16 -> 2.4MHz delta sigma DAC)
// 4 -> /32 (38.4MHz / 32 -> 1.2MHz delta sigma DAC)
// 5 -> /64 (38.4MHz / 64 -> 600KHz delta sigma DAC)

#define CONFIG_SPI_CLK_DIV 3

#define SYSTICK_IRQ_PRIORITY 12

#define FULL_DMATX_SIZE 32
#define FULL_DMARX_SIZE 32

#define APB_CLK_FREQ 24000000

#include "custom_buildconf.h"

#define CONFIG_DMA_PORT_TX 1
#define CONFIG_DMA_TX_CHANNEL 5

#define CONFIG_DMA_PORT_RX 2
#define CONFIG_DMA_RX_CHANNEL 1

#define PAST2(x,y) x ## y
#define EVAL2(x,y)  PAST2(x,y)

#define PAST3(x,y,z) x ## y ## z
#define EVAL3(x,y,z)  PAST3(x,y,z)

#define PAST4(a,b,c,d) a ## b ## c ## d
#define EVAL4(a,b,c,d)  PAST4(a,b,c,d)

#define PAST5(a,b,c,d,e) a ## b ## c ## d ## e
#define EVAL5(a,b,c,d,e) PAST5(a,b,c,d,e)

#define CONFIG_SPI_CTRL         EVAL2(SPI, CONFIG_SPI_PORT)

#define CONFIG_DMA_CTRL_TX      EVAL2(DMA, CONFIG_DMA_PORT_TX)
#define CONFIG_DMA_CTRL_RX      EVAL2(DMA, CONFIG_DMA_PORT_RX)

#define CONFIG_DMA_CHN_TX_CTRL  EVAL4(DMA, CONFIG_DMA_PORT_TX, _Stream, CONFIG_DMA_TX_CHANNEL)
#define CONFIG_DMA_CHN_RX_CTRL  EVAL4(DMA, CONFIG_DMA_PORT_RX, _Stream, CONFIG_DMA_RX_CHANNEL)
#define CONFIG_DMA_CHN_CTRL_IRQ_TX DMA1_Stream5_IRQn //EVAL5(DMA, CONFIG_DMA_PORT, _Channel, CONFIG_DMA_CHANNEL, _IRQn)
#define CONFIG_DMA_CHN_CTRL_IRQ_RX DMA2_Stream1_IRQn //EVAL5(DMA, CONFIG_DMA_PORT, _Channel, CONFIG_DMA_CHANNEL, _IRQn)

#define DMA_IRQ_ENTRY_NAME_TX   DMA1_Stream5_IRQHandler //EVAL5(DMA, CONFIG_DMA_PORT, _Channel, CONFIG_DMA_CHANNEL, _IRQHandler)
#define DMA_IRQ_ENTRY_NAME_RX   DMA2_Stream1_IRQHandler //EVAL5(DMA, CONFIG_DMA_PORT, _Channel, CONFIG_DMA_CHANNEL, _IRQHandler)

#endif
