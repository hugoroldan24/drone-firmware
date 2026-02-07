
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file pwm.c																 *
 * @brief																	 *
 * Initializes and controls the PWM outputs for drone motors.				 *
 * Configures TIM3 for generating PWM signals and TIM4 as an update timer	 *
 * to synchronize CCR register updates safely.								 *
 * Provides functions to start PWM generation and update motor pulse widths. *
 *																			 *
 * Pins used:																 *
 * PA6: TIM3_CH1															 *
 * PA7: TIM3_CH2													   		 *
 * PB0: TIM3_CH3														     *
 * PB1: TIM3_CH4															 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f411xe.h"
#include "const.h"
#include "dron_structs.h"

static void update_timer_init(void);
static void GPIO_PWM_Init(void);

volatile static uint16_t ccr1,ccr2,ccr3,ccr4;
extern PWM_Outputs_t ccr;  /* Updated in motor_mixer.c with the latest motor PWM values */

/**
 * @brief Initializes TIM4 as an update timer to safely update TIM3 CCR registers.
 */
static void update_timer_init()
{
    /* Enable clock access to TIM4 (used as update timer) */
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    /* Set prescaler: APB1 clock doubled, we want 1 MHz timer */
	TIM4->PSC = 2*APB1_CLK_MHZ - 1;  // 100 MHz / (99 +1) = 1 MHz -> 1 tick = 1us

	/* Upcounting mode*/
	TIM4->CR1 &= ~TIM_CR1_DIR;

    /* Only counter overflow triggers interrupt (avoid unwanted interrupts on UG) */
	TIM4->CR1 |= TIM_CR1_URS;

    /* Set auto-reload period for update timer */
	TIM4->ARR = UPDATE_REG_PERIOD - 1;

    /* Force update event from ARR (counter resets to 0) */
	TIM4->EGR |= TIM_EGR_UG;

    /* Clear any pending interrupt flag */
	TIM4->SR &= ~TIM_SR_UIF;

	/* Enable event interrupt */
	TIM4->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM4_IRQn);  // Enable TIM4 interrupt in NVIC
}


/**
 * @brief Configures GPIO pins PA6, PA7, PB0, PB1 for alternate function TIM3 PWM output.
 */
static void GPIO_PWM_Init()
{
    /* Set PA6, PA7, PB0, PB1 as alternate function mode for TIM3 */
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER6) | (2U << GPIO_MODER_MODER6_Pos);
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER7) | (2U << GPIO_MODER_MODER7_Pos);
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER0) | (2U << GPIO_MODER_MODER0_Pos);
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER1) | (2U << GPIO_MODER_MODER1_Pos);

    /* Set alternate function AF2 for TIM3 channels */
	GPIOA->AFR[0] = ((GPIOA->AFR[0])&~(GPIO_AFRL_AFSEL6)) | (2U << GPIO_AFRL_AFSEL6_Pos);
	GPIOA->AFR[0] = ((GPIOA->AFR[0])&~(GPIO_AFRL_AFSEL7)) | (2U << GPIO_AFRL_AFSEL7_Pos);
	GPIOB->AFR[0] = ((GPIOB->AFR[0])&~(GPIO_AFRL_AFSEL0)) | (2U << GPIO_AFRL_AFSEL0_Pos);
	GPIOB->AFR[0] = ((GPIOB->AFR[0])&~(GPIO_AFRL_AFSEL1)) | (2U << GPIO_AFRL_AFSEL1_Pos);
}

/**
 * @brief Initializes PWM outputs on TIM3.
 *        Sets PWM mode, preload registers, initial CCR values, and update generation.
 */
void pwm_init()
{
    /* Configure PWM GPIO pins */
	GPIO_PWM_Init();

    /* Enable clock access to TIM3 (generates PWM signals) */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Set prescaler: 1 tick = 1 us */
	TIM3->PSC = 2*APB1_CLK_MHZ - 1;  // 100 MHz / (99 +1) = 1 MHz -> 1 tick = 1us

    /* Upcounting mode */
	TIM3->CR1 &= ~TIM_CR1_DIR;

    /* Set PWM mode 1 for channels 1-4 */
	TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_OC1M) | (6U << TIM_CCMR1_OC1M_Pos);
	TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_OC2M) | (6U << TIM_CCMR1_OC2M_Pos);
	TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC3M) | (6U << TIM_CCMR2_OC3M_Pos);
	TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC4M) | (6U << TIM_CCMR2_OC4M_Pos);

    /* Enable preload registers for all channels to avoid immediate update on CCR change */
	TIM3->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
	TIM3->CCMR2 |= (TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE);

    /* Set PWM period (ARR) in microseconds */
	TIM3->ARR = ESC_PWM_PERIOD -1;

    /* Set initial CCR values (pulse width) to IDLE_PULSE */
	TIM3->CCR1 = (uint16_t)IDLE_PULSE;
	TIM3->CCR2 = (uint16_t)IDLE_PULSE;
	TIM3->CCR3 = (uint16_t)IDLE_PULSE;
	TIM3->CCR4 = (uint16_t)IDLE_PULSE;

    /* Force update event to load CCR and ARR values into shadow registers */
	TIM3->EGR |= TIM_EGR_UG;

    /* Enable PWM output channels (active high) */
	TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

    /* Initialize update timer TIM4 to safely update CCR registers */
	update_timer_init();
}

/**
 * @brief Starts PWM generation on TIM3 and enables TIM4 update timer.
 */
void start_PWM(void)
{
    /* Start TIM3 to generate PWM signals */
	TIM3->CR1 |= TIM_CR1_CEN;

    /* Start TIM4 to synchronize CCR updates */
	TIM4->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief TIM4 interrupt handler.
 *        Updates TIM3 CCR registers with the latest PWM values from the `ccr` struct.
 */
void TIM4_IRQHandler(void)
{
	if(TIM4->SR & TIM_SR_UIF)
	{
        /* Clear update interrupt flag */
		TIM4->SR &= ~TIM_SR_UIF;

        /* Update TIM3 CCR registers with latest motor PWM values */
		TIM3->CCR1 = ccr.motor1_pwm;
		TIM3->CCR2 = ccr.motor2_pwm;
		TIM3->CCR3 = ccr.motor3_pwm;
		TIM3->CCR4 = ccr.motor4_pwm;
	}
}



