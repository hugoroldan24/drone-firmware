/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file system_init.c												         *
 * @brief																	 *
 * This module configures the system clock to 100 MHz using the HSE and PLL, *
 * enables access to GPIO clocks, grants access to the FPU, and sets NVIC	 *
 * interrupt priorities. All functions defined here are invoked inside		 *
 * SystemInit(), which is executed automatically from the Reset_Handler		 *
 * before main().															 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f411xe.h"
#include <stdint.h>

static void SystemClock_Init(void);
static void ClockAcces_Init(void);
static void FPU_Enable(void);
static void NVIC_Prio_Config(void);

void SystemInit()
{
	SystemClock_Init();
	ClockAcces_Init();
	FPU_Enable();
	NVIC_Prio_Config();
}

/**
 * @brief  Initializes system clocks and sets PLL configuration.
 *
 * Configures the external HSE oscillator, sets flash latency,
 * configures PLL multipliers/dividers to reach 100 MHz system clock,
 * and sets bus prescalers. Switches SYSCLK source to PLL.
 */
static void SystemClock_Init(void)
{
	/* Enable HSE oscillator */
	RCC->CR |= RCC_CR_HSEON;

	/* Wait until HSE is ready */
	while (!(RCC->CR & RCC_CR_HSERDY));

	/* Configure 3 wait states (100 MHz @ 3.3V supply) */
	FLASH->ACR |= FLASH_ACR_LATENCY_3WS;

	/* Select HSE as PLL input source */
	RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSE;

	/* Configure PLLM = 4 → VCO_IN = 8 MHz / 4 = 2 MHz (valid range 1–2 MHz) */
	RCC->PLLCFGR = ((RCC->PLLCFGR) &~(RCC_PLLCFGR_PLLM)) | RCC_PLLCFGR_PLLM_2;

	/* Configure PLLN = 100 → VCO_OUT = 2 MHz * 100 = 200 MHz */
	RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLN) | (100 << RCC_PLLCFGR_PLLN_Pos);

	/* Configure PLLP = 2 → PLLCLK = 200 MHz / 2 = 100 MHz system clock */
	RCC->PLLCFGR = (RCC->PLLCFGR &~RCC_PLLCFGR_PLLP)  | (0 << RCC_PLLCFGR_PLLP_Pos);

	/* Configure APB1 prescaler = /2 → 50 MHz (must be ≤ 50 MHz) */
	RCC->CFGR = (RCC->CFGR &~RCC_CFGR_PPRE1)  | RCC_CFGR_PPRE1_2;

	/* Enable PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait until PLL is ready */
	while(!(RCC->CR & RCC_CR_PLLRDY));

	/* Switch system clock source to PLL */
	RCC->CFGR = (RCC->CFGR &~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;

	/* Wait until PLL is confirmed as system clock source */
	while(((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL));
}

/**
 * @brief  Enables clock access to GPIO ports.
 *
 * Activates AHB1 peripheral clocks for GPIOA, GPIOB, and GPIOC,
 * ensuring these ports can be used by peripherals or firmware.
 */
static void ClockAcces_Init(void)
{
	/* Enable clock access to GPIOA */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	/* Enable clock access to GPIOB */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	/* Enable clock access to GPIOC */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
}

/**
 * @brief  Configures interrupt priorities for NVIC.
 *
 * Sets global priority grouping and assigns specific priorities
 * to peripherals such as TIM4 (PWM), DMA, I2C1, EXTI4, SPI2, and ADC.
 */
static void NVIC_Prio_Config(void)
{
	/* Set priority grouping: 4 bits preemption, 0 subpriority */
	NVIC_SetPriorityGrouping(0x3);

	/* Assign individual priorities (lower = higher priority) */
	NVIC_SetPriority(TIM4_IRQn   	  , 1);
	NVIC_SetPriority(DMA2_Stream7_IRQn, 2);
	NVIC_SetPriority(I2C1_EV_IRQn	  , 3);
	NVIC_SetPriority(EXTI4_IRQn  	  , 3);
	NVIC_SetPriority(SPI2_IRQn   	  , 4);
	NVIC_SetPriority(ADC_IRQn    	  , 4);
	NVIC_SetPriority(DMA2_Stream7_IRQn, 4);
}

/* @brief  Enables floating point unit (FPU).
*
* Grants full access to coprocessors CP10 and CP11, required for
* floating point operations. Uses synchronization barriers to
* ensure proper activation before continuing execution.
*/
static void FPU_Enable(void)
{
	/* Set full access for coprocessors CP10 and CP11 (FPU) */
	SCB->CPACR = (0xF << 20);

	/* Data and instruction barriers to ensure FPU config takes effect */
	__DSB();
	__ISB();
}

