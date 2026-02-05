/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file battery_lecture.c																					 *
 * @brief																								     *
 * This module initializes ADC1 to measure the drone battery voltage on PA2 (ADC channel 2). Conversions are *
 * triggered manually by executing the function start_battery_lecture.									     *
 * Provides functions to initialize the subsystem and obtain the latest ADC result.					         *
 *																											 *
 * Pins used:																								 *
 * PA2																						    		     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f411xe.h"
#include "const.h"
#include "delay.h"
#include "freeRTOS.h"
#include "task.h"
#include <stdint.h>


static void adc_callback(void);
static volatile BaseType_t pxHigherPriorityTaskWoken;
static volatile uint16_t   battery_adc; /* Variable where the latest conversion is stored      */
extern TaskHandle_t SENSORS_ID;  /* Task handle of the task that will execute this code */

/**
 * @brief  Initialize ADC1 for battery measurement on PA2 and enable EOC interrupt.
 *         Configures GPIOA PA2 as analog input, sets up ADC1 regular sequence to sample
 *         channel 2 only, configures ADC common prescaler and sampling time to obtain a
 *         safe conversion window for battery reading, enables the end-of-conversion interrupt
 *         then enables the ADC.
 */
void init_adc(void)
{
	/* Configure PA2 as analog input (ADC channel 2).
		 * Clear PA2 mode bits then set them to '11' for analog mode.
	*/
	GPIOA->MODER = (GPIOA->MODER &~GPIO_MODER_MODER2) | (0x3UL << GPIO_MODER_MODER2_Pos);

	/* Enable clock access to ADC1 */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	/* Set regular sequence length L = 0 (only one conversion in the sequence) */
	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L);

	/* Select ADC channel 2 as the first (and only) conversion in SQR3 (SQ1) */
	ADC1->SQR3 = (ADC1->SQR3 & ~ADC_SQR3_SQ1) | (0x2UL << ADC_SQR3_SQ1_Pos);

	/* Configure ADC common prescaler to scale the APB2 clock into the ADC clock domain.
	 * This aims to set fADC to a value suitable for the chosen sampling time (implemented
	 * here to produce ~25 MHz fADC according to project macros/clock setup).
	 */
	ADC1_COMMON->CCR = (ADC1_COMMON->CCR & ~ADC_CCR_ADCPRE) | (0x1UL << ADC_CCR_ADCPRE_Pos); // APB2/4 = 25 MHz

	/* Configure sampling time for channel 2 to the maximum (480 cycles).
		 * Total conversion time = sampling_cycles + 12 (for 12-bit resolution).
		 * With the selected prescaler, full conversion takes ~20 µs which is acceptable
		 * given the low sampling frequency for battery measurements.
    */
	ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP2) | (0x7UL << ADC_SMPR2_SMP2_Pos); // 480 cycles

	/* Enable End of Conversion (EOC) interrupt */
	ADC1->CR1 |= ADC_CR1_EOCIE;

	/* Enable ADC IRQ in NVIC */
	NVIC_EnableIRQ(ADC_IRQn);

	/* Turn on the ADC */
	ADC1->CR2 |= ADC_CR2_ADON;
}


/**
 * @brief  Start a single mode conversion
 */
void start_battery_lecture(void)
{
	ADC1->CR2 |= ADC_CR2_SWSTART;
}


/**
 * @brief  Blocking read of the latest ADC battery measurement.
 *         Busy-waits until the ADC ISR notifies new data
 *         then returns the last conversion value stored in
 *         `battery_adc`.
 */

 //POTENCIAL BLOQUEO, IMAGINA QUE JUSTO DESPUES DE EMPEZAR A TRANSMITIR, NOS PREEMPTAN,
 //ENTONCES EL ADC CALLBACK ENVIARIA NOTIFICACION, PERO COMO NO HEMOS LLEGADO A BLOQUEARNOS
 //LUEGO NOS BLOQUEAREMOS Y YA NO SE PUEDE DESHACER. SOLUCION: AÑADIR TIME OUT O SEMÁFORO
uint16_t read_battery(void)
{
	start_battery_lecture();
	xTaskNotifyWait(0,0,NULL,pdMS_TO_TICKS(10U)); /* 10 ms timeout to avoid blocking permanently the task */
	return battery_adc;
}


/**
 * @brief  Store ADC result and signal availability.
 */
static void adc_callback(void)
{

	battery_adc = ADC1->DR;

	pxHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(SENSORS_ID,0,eNoAction,&pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

/**
 * @brief  ADC end-of-conversion interrupt handler.
 *         Checks EOC flag and calls the callback to capture the result.
 */
void ADC_IRQHandler(void)
{
	/* Check for EOC (End Of Conversion) in status register */
	if((ADC1->SR & ADC_SR_EOC) != 0){
		adc_callback();
	}
}

