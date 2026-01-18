///////////////////////////////////////////////////////////////////////////////
// File : main.c
// Contains: Uart support
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

volatile int uart_rx_push = 0;
volatile int uart_rx_pop = 0;
volatile unsigned char uart_rx[1024*4] __attribute__((aligned(4)));

void USART2_IRQHandler(void)
{
	if( (USART2->SR & USART_SR_RXNE) )
	{
		uart_rx[uart_rx_push] = USART2->DR;
		uart_rx_push = (uart_rx_push + 1) & (sizeof(uart_rx)-1);
	}
}

unsigned char popserchar()
{
	unsigned char c;

	if( uart_rx_push != uart_rx_pop )
	{
		c = uart_rx[uart_rx_pop];
		uart_rx_pop = (uart_rx_pop + 1) & (sizeof(uart_rx) - 1);

		return c;
	}

	return 0xFF;
}
