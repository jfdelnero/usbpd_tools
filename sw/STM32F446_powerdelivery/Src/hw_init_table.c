///////////////////////////////////////////////////////////////////////////////
//---------------------------------------------------------------------------//
//----------H----H--X----X-----CCCCC-----22222----0000-----0000-----11-------//
//---------H----H----X-X-----C--------------2---0----0---0----0---1-1--------//
//--------HHHHHH-----X------C----------22222---0----0---0----0-----1---------//
//-------H----H----X--X----C----------2-------0----0---0----0-----1----------//
//------H----H---X-----X---CCCCC-----22222----0000-----0000----11111---------//
//---------------------------------------------------------------------------//
//----- Contact: hxc2001 at hxc2001.com ----------- https://hxc2001.com -----//
//----- (c) 2021 Jean-François DEL NERO ----------- http://hxc2001.free.fr --//
///////////////////////////////////////////////////////////////////////////////
// File : hw_init_table.c
// Contains: Hardware descriptors tables and hardware init table executor
//
// Written by: Jean-François DEL NERO
//
// Change History (most recent first):
///////////////////////////////////////////////////////////////////////////////

#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "stm32f4xx_hal_gpio.h"

#include "buildconf.h"

#include "hw_init_table.h"

#include "mini_stm32f4xx_defs.h"

////////////////////////////////////////////////////////////////////////////////////////////////////

#define HW_INIT_TABLE_ID HW_INIT_GLOBAL_INIT // <------
const unsigned long hw_init[] =
{
	0x434F4C42,  ( 0x00005F4B | INT_TO_ASCIIWORD(HW_INIT_TABLE_ID) ),

	/* Disable all clocks interrupts and clear pending bits  */
	(unsigned long)&RCC->CIR,    0x00FF0000,

	(unsigned long)&EXTI->IMR,  0x00000000,
	(unsigned long)&EXTI->RTSR, 0x00000000,
	(unsigned long)&EXTI->FTSR, 0x00000000,

	CMD_INITCLK     ,     0x00000000,

	/* Set the interrupts vectors table base */
	(unsigned long)&SCB->VTOR,    (0x00000000UL),

	/* Set the interrupts vectors table base */
	// -- (unsigned long)&SCB->VTOR,    (FLASH_BASE),

	CMD_MODIFY_DATA,            (unsigned long)&SCB->AIRCR, // HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	                           (SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk),
	                           ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | ((uint32_t)NVIC_PRIORITYGROUP_4 << 8U) ,

	// Systick config
    (unsigned long)&SysTick->LOAD  ,            (uint32_t)( (153000000) - 1UL) , // set reload register
	(unsigned long)&NVIC->IP[SysTick_IRQn]  , (uint8_t)((  TICK_INT_PRIORITY << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL) ,  // set Priority for Systick Interrupt
    (unsigned long)&SCB->SHP[(((uint32_t)(int32_t)SysTick_IRQn) & 0xFUL)-4UL], (uint8_t)((TICK_INT_PRIORITY << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL),
    (unsigned long)&SysTick->VAL                , 0x00000000 ,           // Load the SysTick Counter Value
    (unsigned long)&SysTick->CTRL               , SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk   | SysTick_CTRL_ENABLE_Msk ,   // Enable SysTick IRQ and SysTick Timer

	//(unsigned long)&RCC->APB2ENR                , RCC_APB2ENR_AFIOEN ,   // __HAL_RCC_AFIO_CLK_ENABLE

	// Init cpu systick counter for CMD_DELAY ////////////////////////////////////////////////////////
  CMD_SET,                    (unsigned long)0xE000E018,  // SYST_CVR
                              0x00000000,
  CMD_MODIFY_DATA,            (unsigned long)0xE000E014,  // SYST_RVR
                              0x00FFFFFF,
                              0x00FFFFFF,
  CMD_SET,                    (unsigned long)0xE000E010,  // SYST_CSR
                              0x00000002,                 // enable the counter
	/////////////////////////////////////////////////////////////////////////////////////////////////

  CMD_DELAY         ,   5000,         // Delay

	SET_SYSIRQ_PRI(MemoryManagement_IRQn,0),
	SET_SYSIRQ_PRI(BusFault_IRQn,0),
	SET_SYSIRQ_PRI(UsageFault_IRQn,0),
	SET_SYSIRQ_PRI(SVCall_IRQn,0),
	SET_SYSIRQ_PRI(DebugMonitor_IRQn,0),
	SET_SYSIRQ_PRI(PendSV_IRQn,0),
	SET_SYSIRQ_PRI(SysTick_IRQn,1),

	// PC3 -- CN7
	CMD_MODIFY_DATA,        (unsigned long)&GPIOC->MODER,
							PINCFGMASK(PIN_3),
							0x1 << PINSHIFT(PIN_3), // out
	(unsigned long)&GPIOC->BSRR, (0x0001) << (3+16),

/*
	// PA8 -- MCO1
	(unsigned long)&GPIOA->BSRR, (0x0001) << (PIN_8+16),

	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->OSPEEDR,
							PINCFGMASK(PIN_8),
							0x3 << PINSHIFT(PIN_8), // High speed

	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->PUPDR,
							PINCFGMASK(PIN_8),
							0x0 << PINSHIFT(PIN_8), // No pull up/down

	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->MODER,
							PINCFGMASK(PIN_8),
							0x2 << PINSHIFT(PIN_8), // alt

	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->AFR[1],
							PINALTCFGMASK(PIN_8),
							(( 0 ) << PINALTCFGSHIFT(PIN_8)),
*/

	// PC9 -- MCO2
	(unsigned long)&GPIOC->BSRR, (0x0001) << (PIN_9+16),

	CMD_MODIFY_DATA,        (unsigned long)&GPIOC->OSPEEDR,
							PINCFGMASK(PIN_9),
							0x3 << PINSHIFT(PIN_9), // High speed

	CMD_MODIFY_DATA,        (unsigned long)&GPIOC->PUPDR,
							PINCFGMASK(PIN_9),
							0x0 << PINSHIFT(PIN_9), // No pull up/down

	CMD_MODIFY_DATA,        (unsigned long)&GPIOC->MODER,
							PINCFGMASK(PIN_9),
							0x2 << PINSHIFT(PIN_9), // alt

	CMD_MODIFY_DATA,        (unsigned long)&GPIOC->AFR[1],
							PINALTCFGMASK(PIN_9),
							(( 0 ) << PINALTCFGSHIFT(PIN_9)),

	// PA2 UART TX
	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->MODER,
							PINCFGMASK(PIN_2),
							0x2 << PINSHIFT(PIN_2), // Alt func
	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->AFR[0],
							PINALTCFGMASK(PIN_2),
							(( 7 ) << PINALTCFGSHIFT(PIN_2)),  // USART2_TX

	// PA3 UART RX
	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->MODER,
							PINCFGMASK(PIN_3),
							0x2 << PINSHIFT(PIN_3), // Alt func
	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->AFR[0],
							PINALTCFGMASK(PIN_3),
							(( 7 ) << PINALTCFGSHIFT(PIN_3)),  // USART2_RX

////////////////////////////////////////////////////////////////////////////////////////////////////////

	CMD_CLR,                (unsigned long)&USART2->CR1,
							USART_CR1_UE,               // Disable UART

	CMD_MODIFY_DATA,        (unsigned long)&USART2->CR2,
							USART_CR2_STOP ,
							UART_STOPBITS_1 ,

	CMD_MODIFY_DATA,        (unsigned long)&USART2->CR1,
							(uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE) ,
							UART_WORDLENGTH_8B | UART_PARITY_NONE | UART_MODE_TX_RX ,

	CMD_MODIFY_DATA,        (unsigned long)&USART2->CR3,
							(uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE) ,
							UART_HWCONTROL_NONE,

	(unsigned long)&USART2->BRR, 38,//1000, /*38400*/ // UART_BRR_SAMPLING16( 38400000, 1000000 ) ,

	CMD_CLR,                (unsigned long)&USART2->CR2,
							(USART_CR2_LINEN | USART_CR2_CLKEN),

	CMD_CLR,                (unsigned long)&USART2->CR3,
							(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN),

	SET_IRQ_PRI(USART2_IRQn,1),
	EN_IRQ(USART2_IRQn),

	CMD_SET,                (unsigned long)&USART2->CR1,
							USART_CR1_UE | USART_CR1_RXNEIE , // Enable UART


////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0

	//////////////////////////////////////////////////////////////////////////
	// Init SPI
	//////////////////////////////////////////////////////////////////////////

	// PF0 -- CN3-10-D7
	CMD_MODIFY_DATA,        (unsigned long)&GPIOF->MODER,
							PINCFGMASK(PIN_0),
							0x1 << PINSHIFT(PIN_0), // out
	(unsigned long)&GPIOF->BSRR, (0x0001) << (0+16),

	// PF1 -- CN3-11-D8
	CMD_MODIFY_DATA,        (unsigned long)&GPIOF->MODER,
							PINCFGMASK(PIN_1),
							0x1 << PINSHIFT(PIN_1), // out
	(unsigned long)&GPIOF->BSRR, (0x0001) << (1+16),

	// PA8 - in signal
	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->MODER,
							PINCFGMASK(PIN_8),
							0x2 << PINSHIFT(PIN_8), // Alt func
	(unsigned long)&GPIOA->BSRR, (0x0001) << (PIN_8),
	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->AFR[1],
							PINALTCFGMASK(PIN_8),
							(( 2 ) << PINALTCFGSHIFT(PIN_8)),

	// PB3 LED
	CMD_MODIFY_DATA,        (unsigned long)&GPIOB->MODER,
							PINCFGMASK(PIN_3),
							0x1 << PINSHIFT(PIN_3), // Out
	(unsigned long)&GPIOB->BSRR, (0x0001) << (3+16),





#endif

	// PB10 -- end of frame
	(unsigned long)&GPIOB->BSRR, (0x0001) << (PIN_10+16),

	CMD_MODIFY_DATA,        (unsigned long)&GPIOB->PUPDR,
							PINCFGMASK(PIN_10),
							0x2 << PINSHIFT(PIN_10), // pull down

	CMD_MODIFY_DATA,        (unsigned long)&GPIOB->MODER,
							PINCFGMASK(PIN_10),
							0x0 << PINSHIFT(PIN_10), // input

	// PA8 -- Stream in
	(unsigned long)&GPIOA->BSRR, (0x0001) << (PIN_8+16),

	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->PUPDR,
							PINCFGMASK(PIN_8),
							0x1 << PINSHIFT(PIN_8), // pull up

	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->MODER,
							PINCFGMASK(PIN_8),
							0x2 << PINSHIFT(PIN_8), // alt

	CMD_MODIFY_DATA,        (unsigned long)&GPIOA->AFR[1],
							PINALTCFGMASK(PIN_8),
							(( 1 ) << PINALTCFGSHIFT(PIN_8)), // TIM1_CH1


	// PB5 SPI MOSI -- CN3-14-D11
	CMD_MODIFY_DATA,        (unsigned long)&GPIOB->MODER,
							PINCFGMASK(PIN_5),
							0x2 << PINSHIFT(PIN_5), // Alt func

	CMD_MODIFY_DATA,        (unsigned long)&GPIOB->AFR[0],
							PINALTCFGMASK(PIN_5),
							(( 6 ) << PINALTCFGSHIFT(PIN_5)),  // SPI3_MOSI

	// PB4 SPI MISO -- CN3-15-D12
	CMD_MODIFY_DATA,        (unsigned long)&GPIOB->MODER,
							PINCFGMASK(PIN_4),
							0x0 << PINSHIFT(PIN_4), // Input exti

	CMD_MODIFY_DATA,        (unsigned long)&GPIOB->AFR[0],
							PINALTCFGMASK(PIN_4),
							(( 6 ) << PINALTCFGSHIFT(PIN_4)),  // SPI3_MISO

	// PB3 SPI SCLK -- CN4-15-D13
	CMD_MODIFY_DATA,        (unsigned long)&GPIOB->MODER,
							PINCFGMASK(PIN_3),
							0x2 << PINSHIFT(PIN_3), // Alt func
	CMD_MODIFY_DATA,        (unsigned long)&GPIOB->AFR[0],
							PINALTCFGMASK(PIN_3),
							(( 6 ) << PINALTCFGSHIFT(PIN_3)),  // SPI3_SCK


	(unsigned long)&CONFIG_SPI_CTRL->CR2, 0x00000000, /*SPI_CR2_TXDMAEN,*/
	(unsigned long)&CONFIG_SPI_CTRL->CR1,  (SPI_CR1_SSI | SPI_CR1_SSM) | (( ((CONFIG_SPI_CLK_DIV&7)<<3) | SPI_CR1_MSTR)),

	SET_IRQ_PRI(CONFIG_DMA_CHN_CTRL_IRQ_TX,7),
	EN_IRQ(CONFIG_DMA_CHN_CTRL_IRQ_TX),

	SET_IRQ_PRI(CONFIG_DMA_CHN_CTRL_IRQ_RX,6),
	EN_IRQ(CONFIG_DMA_CHN_CTRL_IRQ_RX),

	(unsigned long)&CONFIG_DMA_CHN_RX_CTRL->CR, 0x00000000,
	(unsigned long)&CONFIG_DMA_CHN_RX_CTRL->NDTR, 0x00000000,

	(unsigned long)&CONFIG_DMA_CHN_TX_CTRL->CR, 0x00000000,
	(unsigned long)&CONFIG_DMA_CHN_TX_CTRL->NDTR, 0x00000000,

	(unsigned long)&CONFIG_DMA_CTRL_TX->LIFCR, 0x00000FF0,

	// Configure DMA Channel data length

	(unsigned long)&CONFIG_DMA_CHN_TX_CTRL->NDTR, (FULL_DMATX_SIZE / 2),
	(unsigned long)&CONFIG_DMA_CHN_TX_CTRL->PAR, (uint32_t)&CONFIG_SPI_CTRL->DR,

	(unsigned long)&CONFIG_DMA_CHN_TX_CTRL->CR, DMA_MEMORY_TO_PERIPH | DMA_PINC_DISABLE | DMA_MINC_ENABLE | \
										DMA_PDATAALIGN_BYTE | DMA_MDATAALIGN_BYTE | DMA_CIRCULAR | DMA_PRIORITY_VERY_HIGH | \
										(DMA_IT_TC | DMA_IT_HT | DMA_IT_TE) | (0<<25),

	(unsigned long)&CONFIG_DMA_CHN_RX_CTRL->NDTR, (FULL_DMARX_SIZE),
	(unsigned long)&CONFIG_DMA_CHN_RX_CTRL->PAR, (uint32_t)&TIM1->CCR1,

	(unsigned long)&CONFIG_DMA_CHN_RX_CTRL->CR, DMA_PERIPH_TO_MEMORY | DMA_PINC_DISABLE | DMA_MINC_ENABLE | \
										DMA_PDATAALIGN_HALFWORD | DMA_MDATAALIGN_HALFWORD | DMA_CIRCULAR | DMA_PRIORITY_MEDIUM | \
										(DMA_IT_TC | DMA_IT_HT | DMA_IT_TE) | (6<<25),

#if 0
	// ADC
	CMD_SET,                (unsigned long)&ADC1_COMMON->CCR,
							ADC_CCR_TSEN | ADC_CCR_VBATEN | ADC_CCR_VREFEN,

	CMD_SET,                (unsigned long)&ADC1->CR,
							ADC_CR_ADCAL,
	CMD_WAITBITCLR,         (unsigned long)&ADC1->CR, ADC_CR_ADCAL, // Wait for calibration


	(unsigned long)&ADC1->SMPR, 0x6, // Sampling time

	(unsigned long)&ADC1->CHSELR, (0x1<<18)|(0x1<<17)|(0x1<<16)|(0x1<<1)|(0x1<<0), // Sampling time

	//CMD_SET,                (unsigned long)&ADC1->CFGR1,
	//                      ADC_CFGR1_DISCEN,

	CMD_SET,                (unsigned long)&ADC1->CR,
							ADC_CR_ADEN,
	CMD_WAITBITSET,         (unsigned long)&ADC1->ISR, ADC_ISR_ADRDY, // Wait ready

	CMD_CLR,                (unsigned long)&TIM14->CR1,TIM_CR1_UDIS,
	(unsigned long)&TIM14->ARR,  38200, // Period
	(unsigned long)&TIM14->PSC,  0,      // Prescaler (24MHz / 1 = 24MHz)
	(unsigned long)&TIM14->EGR,  TIM_EGR_UG, //Generate an update event to reload the Prescaler.
	CMD_SET,                    (unsigned long)&TIM14->DIER, TIM_IT_UPDATE,  // IT enabled
	CMD_SET,                    (unsigned long)&TIM14->CR1, TIM_CR1_CEN,  // Enable

	CMD_MODIFY_DATA,        (unsigned long)&NVIC->IP[((uint32_t)(int32_t)TIM14_IRQn >> 2UL)], (0xFF<<((TIM14_IRQn&3)*8)), 0xE0,
	(unsigned long)&NVIC->ISER[(((uint32_t)TIM14_IRQn) >> 5UL)], (uint32_t)(1UL << ((uint32_t)(TIM14_IRQn& 0x1FUL))),

	CMD_CLR,                (unsigned long)&TIM16->CR1,TIM_CR1_UDIS,
	(unsigned long)&TIM16->ARR,  1600, // Period
	(unsigned long)&TIM16->PSC,  0,      // Prescaler (24MHz / 1 = 24MHz)
	(unsigned long)&TIM16->EGR,  TIM_EGR_UG, //Generate an update event to reload the Prescaler.
	CMD_CLR,                    (unsigned long)&TIM16->DIER, TIM_IT_UPDATE,  // IT enabled
	CMD_SET,                    (unsigned long)&TIM16->CR1, TIM_CR1_CEN,  // Enable

	CMD_MODIFY_DATA,        (unsigned long)&NVIC->IP[((uint32_t)(int32_t)TIM16_IRQn >> 2UL)], (0xFF<<((TIM16_IRQn&3)*8)), 0x30,
	(unsigned long)&NVIC->ISER[(((uint32_t)TIM16_IRQn) >> 5UL)], (uint32_t)(1UL << ((uint32_t)(TIM16_IRQn& 0x1FUL))),

#endif

	(unsigned long)&TIM1->ARR,  1024, // Period

	(unsigned long)&TIM1->PSC,  7,      // Prescaler (38.4MHz / 2 = 19.2MHz)
	(unsigned long)&TIM1->RCR,  0,      // Repetition
	(unsigned long)&TIM1->EGR,  TIM_EGR_UG, //Generate an update event to reload the Prescaler.

	//Disable slave mode to clock the prescaler directly with the internal clock
	CMD_CLR,                    (unsigned long)&TIM1->SMCR, TIM_SMCR_SMS | (TIM_SMCR_SMS | TIM_SMCR_TS) | (TIM_SMCR_ETF | TIM_SMCR_ETPS | TIM_SMCR_ECE | TIM_SMCR_ETP), // Reset the SMS, TS, ECE, ETPS and ETRF bits

	CMD_CLR,                    (unsigned long)&TIM1->CCER, TIM_CCER_CC1E,  // Disable the channel 1

    //Channel is configured as input, IC1 is mapped on TRC. This mode is working only if
    // an internal trigger input is selected through TS bit (TIMx_SMCR register)
	CMD_MODIFY_DATA,            (unsigned long)&TIM1->CCMR1, (TIM_CCMR1_CC1S | TIM_CCMR1_IC1F) ,  TIM_ICSELECTION_TRC | ((9 << 4) & TIM_CCMR1_IC1F), // Selection + Filter

	CMD_MODIFY_DATA,            (unsigned long)&TIM1->CCER,  (TIM_CCER_CC1P | TIM_CCER_CC1NP) ,  (TIM_ICPOLARITY_BOTHEDGE & (TIM_CCER_CC1P | TIM_CCER_CC1NP)), // Polarity + enable

    // TI1 Edge Detector (TI1F_ED) + Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter
    // and generates an update of the registers.
	CMD_MODIFY_DATA,            (unsigned long)&TIM1->SMCR,  (TIM_SMCR_TS | TIM_SMCR_SMS) ,  TIM_TS_TI1F_ED | TIM_SLAVEMODE_RESET, // Input trigger

	CMD_CLR,                    (unsigned long)&TIM1->DIER, TIM_IT_TRIGGER,  // Disable Trigger Interrupt
	CMD_CLR,                    (unsigned long)&TIM1->DIER, TIM_DMA_TRIGGER, // Disable Trigger DMA

	(unsigned long)&TIM1->ARR,  1024, // Period

	CMD_CLR,                    (unsigned long)&TIM1->CR1, TIM_CR1_UDIS, // turn off UDIS
	//CMD_CLR,                    (unsigned long)&TIM1->CR1, TIM_CR1_ARPE, // turn off UDIS

	//SET_IRQ_PRI(TIM1_UP_TIM10_IRQn,20),
	//EN_IRQ(TIM1_UP_TIM10_IRQn),

	CMD_SET,                    (unsigned long)&TIM1->CR1, TIM_CR1_CEN, // turn on TIM1


	// Exti gpio port B pin 10
	CMD_MODIFY_DATA,        (unsigned long)&SYSCFG->EXTICR[PIN_10 >> 2],
							((uint32_t)0x0F) << (4 * (PIN_10 & 0x03)) ,
							(1) << (4 * (PIN_10 & 0x03)) ,

	//CMD_SET,                (unsigned long)&EXTI->RTSR, EXTI_RTSR_TR4,
	CMD_SET,                (unsigned long)&EXTI->FTSR, EXTI_FTSR_TR10,
	CMD_SET,                (unsigned long)&EXTI->IMR,  EXTI_IMR_MR10,

	SET_IRQ_PRI(EXTI15_10_IRQn,4),
	EN_IRQ(EXTI15_10_IRQn),

	0x00000000,

	0x42444E45,  ( 0x00004B4C | INT_TO_ASCIIWORD(HW_INIT_TABLE_ID) )   // "ENDBLKxx"
};
#undef HW_INIT_TABLE_ID

#define HW_INIT_TABLE_ID HW_INIT_CLOCK_INIT // <------
const unsigned long clk_init[] =
{
	0x434F4C42,  ( 0x00005F4B | INT_TO_ASCIIWORD(HW_INIT_TABLE_ID) ) , // "BLOCK_xx"

	CMD_SET,                    (unsigned long)&RCC->APB1ENR, RCC_APB1ENR_PWREN,  //

	CMD_SET,                    (unsigned long)&RCC->BDCR, RCC_BDCR_LSEMOD,  //
	CMD_CLR,                    (unsigned long)&RCC->BDCR, RCC_BDCR_LSEBYP,  //

	CMD_SET,                    (unsigned long)&RCC->BDCR, RCC_BDCR_LSEON,  //
	//CMD_WAITBITSET,             (unsigned long)&RCC->BDCR, RCC_BDCR_LSERDY, //

	CMD_SET,                    (unsigned long)&RCC->CR, RCC_CR_HSION,  // Start 16 MHz HSI
	CMD_WAITBITSET,             (unsigned long)&RCC->CR, RCC_CR_HSIRDY, // Wait for HSI

	// Switch clock source to HSI
	CMD_MODIFY_DATA,            (unsigned long)&RCC->CFGR, RCC_CFGR_SW, RCC_SYSCLKSOURCE_HSI,
	CMD_WAITWORD,               (unsigned long)&RCC->CFGR, RCC_CFGR_SWS, RCC_SYSCLKSOURCE_STATUS_HSI,

	CMD_SET,                    (unsigned long)&RCC->CR, RCC_CR_HSEON,  // Start 8 MHz HSE
	CMD_WAITBITSET,             (unsigned long)&RCC->CR, RCC_CR_HSERDY, // Wait for HSE

	CMD_CLR,                    (unsigned long)&RCC->CR, RCC_CR_PLLON,  // Disable the PLL
	CMD_WAITBITCLR,             (unsigned long)&RCC->CR, RCC_CR_PLLRDY, // Wait for pll disabled

	CMD_SET,                    (unsigned long)&RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC,  // HSE to PLL

	// Setup the PLL
    //f(VCO clock) = f (PLL clock input) × (PLLN / PLLM)
    // 8/4 * (153/2) = 153 MHz
	CMD_MODIFY_DATA,            (unsigned long)&RCC->PLLCFGR, (RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLR | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN) ,
                                    (0<<16)| // PLLP[1:0]: Main PLL (PLL) division factor for main system clock 00: PLLP = 2,01: PLLP = 4,10: PLLP = 6,11: PLLP = 8...
                                    (2<<28)| // PLLR[2:0]: Main PLL division factor for I2Ss, SAIs, SYSTEM and SPDIF-Rx clock 010: PLLR = 2 011: PLLR = 3, ...
                                    (8<<24)| // PLLQ[3:0]: Main PLL (PLL) division factor for USB OTG FS, SDIOclocks 0010: PLLQ = 2 0011: PLLQ = 3 0100: PLLQ = 4..
                                    (4<<0) | // PLLM[5:0]: Division factor for the main PLL (PLL) input clock 000010: PLLM = 2 000011: PLLM = 3 000100: PLLM = 4...
                                    (153<<6), // PLLN[8:0]: Main PLL (PLL) multiplication factor for VCO 000110010: PLLN = 50 .. 432
                                   // ((8/4) * 153) = 306 MHz

                                    // Main system clock : 153 Mhz

	CMD_SET,                    (unsigned long)&RCC->CR, RCC_CR_PLLON,  // Enable the PLL !
	CMD_WAITBITSET,             (unsigned long)&RCC->CR, RCC_CR_PLLRDY, // Wait for pll ready !

// The maximum frequency of the AHB domain is
// 180 MHz. The maximum allowed frequency of the high-speed APB2 domain is 90 MHz. The
// maximum allowed frequency of the low-speed APB1 domain is 45 MHz

	CMD_MODIFY_DATA,            (unsigned long)&RCC->CFGR, RCC_CFGR_HPRE, 0<<4,                 // AHB prescaler : No division (Max 180MHz) 153 MHz
	CMD_MODIFY_DATA,            (unsigned long)&RCC->CFGR, RCC_CFGR_PPRE1, 5<<10,               // APB Low speed prescaler (APB1) MAX45MHz  153/4:38,25Mhz
	CMD_MODIFY_DATA,            (unsigned long)&RCC->CFGR, RCC_CFGR_PPRE2, 4<<13,               // APB high-speed prescaler (APB2) MAX90MHz 153/2:76,5Mhz

	CMD_SET,                    (unsigned long)&FLASH->ACR, FLASH_ACR_ICRST,
	CMD_CLR,                    (unsigned long)&FLASH->ACR, FLASH_ACR_ICRST,

	//CMD_SET,                    (unsigned long)&FLASH->ACR, FLASH_ACR_ICEN,

	CMD_MODIFY_DATA,            (unsigned long)&FLASH->ACR, FLASH_ACR_LATENCY , FLASH_LATENCY_4,
	CMD_SET,                    (unsigned long)&FLASH->ACR, FLASH_ACR_PRFTEN,           // __HAL_FLASH_PREFETCH_BUFFER_ENABLE

	// Switch clock source to PLL
	CMD_MODIFY_DATA,            (unsigned long)&RCC->CFGR, RCC_CFGR_SW, 2<<0,
	CMD_WAITWORD,               (unsigned long)&RCC->CFGR, RCC_CFGR_SWS, 2<<2,

	CMD_MODIFY_DATA,            (unsigned long)&RCC->CFGR, RCC_CFGR_MCO1, 3<<21,                   // MCO1 / PA8 = PLL
	CMD_MODIFY_DATA,            (unsigned long)&RCC->CFGR, RCC_CFGR_MCO2, 0<<30,                   // MCO2 / PC9 = System clock

	CMD_MODIFY_DATA,            (unsigned long)&RCC->CFGR, RCC_CFGR_MCO2PRE, 7<<27,                   // /5
	CMD_MODIFY_DATA,            (unsigned long)&RCC->CFGR, RCC_CFGR_MCO1PRE, 7<<24,                   // /5


	CMD_SET,                    (unsigned long)&SysTick->CTRL, SYSTICK_CLKSOURCE_HCLK,                      // Systick on the host clock

	CMD_SET,                    (unsigned long)&RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN,         // SYSCFG

	CMD_SET,                    (unsigned long)&RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN,    // GPIO ports
	CMD_SET,                    (unsigned long)&RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN, // Enable DMA1 block clock
	CMD_SET,                    (unsigned long)&RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN, // Enable DMA1 block clock

	// --CMD_SET,                    (unsigned long)&RCC->APB2ENR, RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN ,  // Enable ports Clock
	CMD_SET,                    (unsigned long)&RCC->APB2ENR, RCC_APB2ENR_SPI1EN,     // Enable SPI 2 Clock
	CMD_CLR,                    (unsigned long)&RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST,  // SPI 1 reset

	CMD_SET,                    (unsigned long)&RCC->APB1ENR, RCC_APB1ENR_SPI2EN,     // Enable SPI 2 Clock
	CMD_CLR,                    (unsigned long)&RCC->APB1RSTR, RCC_APB1RSTR_SPI2RST,  // SPI 1 reset

	CMD_SET,                    (unsigned long)&RCC->APB1ENR, RCC_APB1ENR_SPI3EN,     // Enable SPI 2 Clock
	CMD_CLR,                    (unsigned long)&RCC->APB1RSTR, RCC_APB1RSTR_SPI3RST,  // SPI 1 reset

	CMD_SET,                    (unsigned long)&RCC->APB1ENR, RCC_APB2ENR_SPI4EN,     // Enable SPI 2 Clock
	CMD_CLR,                    (unsigned long)&RCC->APB1RSTR, RCC_APB2RSTR_SPI4RST,  // SPI 1 reset

	CMD_SET,                    (unsigned long)&RCC->APB2ENR, RCC_APB2ENR_USART1EN,         // UART
	CMD_CLR,                    (unsigned long)&RCC->APB2RSTR, RCC_APB2RSTR_USART1RST,      // UART reset

	CMD_SET,                    (unsigned long)&RCC->APB1ENR, RCC_APB1ENR_USART2EN,         // UART
	CMD_CLR,                    (unsigned long)&RCC->APB1RSTR, RCC_APB1RSTR_USART2RST,      // UART reset

	CMD_SET,                    (unsigned long)&RCC->APB1ENR, RCC_APB1ENR_USART3EN,         // UART
	CMD_CLR,                    (unsigned long)&RCC->APB1RSTR, RCC_APB1RSTR_USART3RST,      // UART reset

	CMD_SET,                    (unsigned long)&RCC->APB2ENR, RCC_APB2ENR_ADC1EN,         // ADC
	CMD_CLR,                    (unsigned long)&RCC->APB2RSTR, RCC_APB2RSTR_ADCRST,      // ADC reset

	CMD_SET,                    (unsigned long)&RCC->APB1ENR, RCC_APB1ENR_TIM14EN,         // TIM14
	CMD_CLR,                    (unsigned long)&RCC->APB1RSTR, RCC_APB1RSTR_TIM14RST,      // TIM14 reset

	CMD_SET,                    (unsigned long)&RCC->APB2ENR, RCC_APB2ENR_TIM11EN,         // TIM11
	CMD_CLR,                    (unsigned long)&RCC->APB2RSTR, RCC_APB2RSTR_TIM11RST,      // TIM11 reset

	CMD_SET,                    (unsigned long)&RCC->APB2ENR, RCC_APB2ENR_TIM1EN,         // TIM1
	CMD_CLR,                    (unsigned long)&RCC->APB2RSTR, RCC_APB2RSTR_TIM1RST,      // TIM1 reset

	CMD_SET,                    (unsigned long)&RCC->APB1ENR, RCC_APB1ENR_TIM2EN,         // TIM2
	CMD_CLR,                    (unsigned long)&RCC->APB1RSTR, RCC_APB1RSTR_TIM2RST,      // TIM2 reset

	CMD_SET,                    (unsigned long)&RCC->APB1ENR, RCC_APB1ENR_TIM3EN,         // TIM3
	CMD_CLR,                    (unsigned long)&RCC->APB1RSTR, RCC_APB1RSTR_TIM3RST,      // TIM3 reset

	CMD_SET,                    (unsigned long)&SysTick->CTRL, SYSTICK_CLKSOURCE_HCLK,                      // Systick on the host clock

	0x00000000,

	0x42444E45,  ( 0x00004B4C | INT_TO_ASCIIWORD(HW_INIT_TABLE_ID) )   // "ENDBLKxx"
};
#undef HW_INIT_TABLE_ID



// |TTttOOOO|mmmmMMMM|Address (0-4)|DATA1(0-4)|DATA2(0-4)|
//             (ext)
// TT : 00 Normal, 01 : With byte masks
// tt : 00 Same address 01 Byte offset 10 Short Offset 11 Long Offset
// mmmm: DATA1 byte mask
// MMMM: DATA2 byte mask

typedef struct _decode_op_stat
{
	volatile uint32_t * current_address;
	uint32_t current_data;
	uint32_t current_mask;
	unsigned char op;
}decode_op_stat;

unsigned int get_cpu_tick()
{
	return *((volatile unsigned int *)0xE000E018); // SYST_CVR
}

unsigned int get_elapsed_cpu_tick(unsigned int start)
{
	unsigned int curcount;

	curcount = *((volatile unsigned int *)0xE000E018); // SYST_CVR

	if( curcount > start )
	{
		return ((0xFFFFFF - start) + curcount);
	}
	else
	{
		return (start - curcount);
	}
}

void uswait(unsigned int us)
{
	unsigned int start,total_time;

	total_time = 72 * us;
	start = *((volatile unsigned int *)0xE000E018); // SYST_CVR
	while(get_elapsed_cpu_tick(start) < total_time);
}

static const unsigned char * __attribute__ ((optimize("-Os"))) update_var(const unsigned char * opcode_buffer, uint32_t * var,unsigned int mask)
{
	int i;

	i = 4;
	while(i--)
	{
		if((0x01<<i) & mask)
		{
			*var &= ~(0xFF << (i*8));
			*var |= (*opcode_buffer++ << (i*8));
		}
	}

	return opcode_buffer;
}

int __attribute__ ((optimize("-Os"))) decode_opcode(const unsigned char * opcode_buffer, decode_op_stat * op_stat)
{
	unsigned char datamask;
	unsigned char addressmask;
	unsigned char opcode;
	uint32_t      address;
	const unsigned char * start_ptr;

	/*
	int i;
	printf("%.8X :",opcode_buffer);

	for(i=0;i<8;i++)
		printf("%.2X ",opcode_buffer[i]);

		printf("\n");
	*/

	start_ptr = opcode_buffer;

	datamask = 0xF0;
	addressmask = 0x0F;

	opcode = *opcode_buffer;
	opcode_buffer++;

	op_stat->op = opcode & 0xF;

	if( ( op_stat->op == (CMD_WAITWORD&0xF) ) || ( op_stat->op == (CMD_MODIFY_DATA&0xF) ) )
		datamask = 0xFF;

	if(opcode & 0xC0)
	{
		datamask = *opcode_buffer;
		opcode_buffer++;
	}

	if( ( op_stat->op == (CMD_READ_DATA&0xF) ) || ( op_stat->op == (CMD_INITCLK&0xF) ) )
		datamask = 0x00;

	// tt : 00 Same address 01 Byte offset 10 Short Offset 11 Long Offset
	addressmask = (opcode>>4)&0x3;
	if(addressmask == 0x3)
		addressmask = 0xF;

	if(addressmask == 0x2)
		addressmask = 0x3;

	address = 0x00000000;

	if((*opcode_buffer & 0x80) && addressmask)
	{
		address = 0xFFFFFFFF;
	}

	opcode_buffer = update_var(opcode_buffer, &address,addressmask);

	op_stat->current_address = (void*)((unsigned char *)op_stat->current_address) + (int)address;

	opcode_buffer = update_var(opcode_buffer, &op_stat->current_data,datamask>>4);
	opcode_buffer = update_var(opcode_buffer, &op_stat->current_mask,datamask);

	return opcode_buffer-start_ptr;
}

//#define PRINT_NEW_TABLE 1

void __attribute__ ((optimize("-Os"))) exec_hw_init_table(unsigned int table_id)
{
	decode_op_stat stat;
	//volatile unsigned int data;
	const unsigned char * table;

	table = (const unsigned char *)&packed_hw_init;
	table += *(((unsigned short*)&packed_hw_init) + table_id);

	stat.current_address = (volatile uint32_t *)0x00000000;
	stat.current_data    = 0x00000000;
	stat.current_mask    = 0x00000000;
	stat.op = 0x00;

	while(1)
	{
		table += decode_opcode(table, &stat);
		switch(stat.op)
		{
			case (CMD_DELAY&0xF):
				#ifdef PRINT_NEW_TABLE
				printf("CMD_DELAY       : %d uS\n",stat.current_data);
				#endif
				uswait(stat.current_data);
			break;
			case (CMD_WAITBITCLR&0xF):
				#ifdef PRINT_NEW_TABLE
				printf("CMD_WAITBITCLR  : ADDRESS 0x%.8X MASK 0x%.8X\n",stat.current_address,stat.current_data);
				#endif
				while ( (*stat.current_address & stat.current_data) );
			break;
			case (CMD_WAITBITSET&0xF):
				#ifdef PRINT_NEW_TABLE
				printf("CMD_WAITBITSET  : ADDRESS 0x%.8X MASK 0x%.8X\n",stat.current_address,stat.current_data);
				#endif

				while ( !(*stat.current_address & stat.current_data) );
			break;
			case (CMD_CLR&0xF):
				#ifdef PRINT_NEW_TABLE
				printf("CMD_CLR         : ADDRESS 0x%.8X DATA 0x%.8X\n",stat.current_address,stat.current_data);
				#endif
				*stat.current_address &= (~stat.current_data);
			break;
			case (CMD_SET&0xF):
				#ifdef PRINT_NEW_TABLE
				printf("CMD_SET         : ADDRESS 0x%.8X DATA 0x%.8X\n",stat.current_address,stat.current_data);
				#endif
				*stat.current_address |= (stat.current_data);
			break;
			case (CMD_READ_DATA&0xF):
				#ifdef PRINT_NEW_TABLE
				printf("CMD_READ_DATA   : ADDRESS 0x%.8X\n",stat.current_address);
				#endif
				//data = *stat.current_address;
				*stat.current_address;
			break;
			case (CMD_MODIFY_DATA&0xF):
				#ifdef PRINT_NEW_TABLE
				printf("CMD_MODIFY_DATA : ADDRESS 0x%.8X MASK 0x%.8X DATA 0x%.8X\n",stat.current_address,stat.current_mask,stat.current_data);
				#endif
				*stat.current_address = (( *stat.current_address & (~stat.current_mask) ) | (stat.current_data & stat.current_mask));
			break;
			case (CMD_WAITWORD&0xF):
				#ifdef PRINT_NEW_TABLE
				printf("CMD_WAITWORD    : ADDRESS 0x%.8X MASK 0x%.8X DATA 0x%.8X\n",stat.current_address,stat.current_mask,stat.current_data);
				#endif
				while ( (*stat.current_address & stat.current_mask) != (stat.current_data & stat.current_mask) );
			break;
			case (CMD_INITCLK&0xF):
				#ifdef PRINT_NEW_TABLE
				printf("CMD_INITCLK\n");
				#endif

				exec_hw_init_table(HW_INIT_CLOCK_INIT); //clk_init
			break;
			case (CMD_WRITE_DATA&0xF):
				#ifdef PRINT_NEW_TABLE
				printf("CMD_WRITE_DATA  : ADDRESS 0x%.8X DATA 0x%.8X\n",stat.current_address,stat.current_data);
				#endif
				*stat.current_address = stat.current_data;
			break;

			default:
				#ifdef PRINT_NEW_TABLE
				printf("bad/end opcode : %x\n",stat.op);
				#endif
				goto end_loop;
			break;
		}
	}
end_loop:
	return;
}
