
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
#include "config.h"
#include "dron_structs.h"

static void update_timer_init(void);
static void GPIO_PWM_Init(void);


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

    /* Set PWM period (ARR) in microseconds */
	TIM3->ARR = ESC_PWM_PERIOD - 1U;

	/*Set One-Pulse mode */
	TIM3->CR1 |= TIM_CR1_OPM;

    /* Force update event to load ARR value into shadow register */
	TIM3->EGR |= TIM_EGR_UG;

    /* Enable PWM output channels (active high) */
	TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
}

/**
 * @brief Updates the PWM output registers of a given timer.
 * 
 * This function writes the PWM duty cycle values to the CCR (Capture/Compare) registers
 * of the specified timer. Each CCR channel corresponds to one motor's ESC on a quadcopter.
 * 
 * @param timer Pointer to the TIM peripheral (e.g., TIM1, TIM3) whose CCR registers
 *              will be updated.
 * @param ccr   Structure containing the PWM duty cycle values for each motor:
 *              - motor1_pwm → CCR1
 *              - motor2_pwm → CCR2
 *              - motor3_pwm → CCR3
 *              - motor4_pwm → CCR4
 * 
 * @note The timer must be properly configured for PWM mode before calling this function.
 *       The values in the CCR registers determine the duty cycle and thus the speed
 *       of each motor via its ESC.
 */
void update_PWM_Outputs(TIM_TypeDef* timer, PWM_Outputs_t ccr)
{
   timer->CCR1 = ccr.motor1_pwm;
   timer->CCR2 = ccr.motor2_pwm;
   timer->CCR3 = ccr.motor3_pwm;
   timer->CCR4 = ccr.motor4_pwm;

   /* Start timer to generate PWM signals (OneShot125) */
   timer->CR1 |= TIM_CR1_CEN;
}



