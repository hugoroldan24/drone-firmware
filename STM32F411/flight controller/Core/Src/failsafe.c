/*************************************************************************
 * @file failsafe.c
 * @brief
 * Manages the drone's failsafe procedures. Handles entering failsafe mode,
 * disabling non-essential peripherals, stabilizing the drone in hover,
 * executing a controlled descent, stopping the motors, and putting the MCU
 * into standby mode for minimal power consumption.
 *************************************************************************/

#include "stm32f411xe.h"
#include "const.h"
#include "dron_structs.h"
#include "FreeRTOS.h"
#include "task.h"

extern void xHandleRCRxTask(void* parameters);
extern TaskHandle_t RC_RX_ID;
extern UserControl_t joystick_data; /* Shared structure holding the latest joystick/control inputs used by the RC_RX task */

/**
 * @brief Executes the full failsafe landing sequence.
 *        Sets descent throttle, waits for the drone to descend, stops motors,
 *        and puts the MCU into standby mode for low power consumption.
 */
void failsafe_execute_landing(void)
{
    /* Set descent throttle to start lowering the drone */
	joystick_data.left_y_axis = DESCENT_THROTTLE;

	/* Wait 20 seconds for the drone to descend */
	vTaskDelay(pdMS_TO_TICKS(20000));

    /* Stop motors completely */
	joystick_data.left_y_axis = NO_THROTTLE;

    /* Configure CPU for deep sleep mode */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set Power Down Deep Sleep (Standby) mode */
	PWR->CR |= PWR_CR_PDDS;

    /* Enter standby mode */
	__WFI();
}

/**
 * @brief Stabilizes the drone in hover.
 *        Deletes and recreates the RC_RX task so that the RC_RX task executes a different code, then waits for stabilization.
 */
void set_hover(void)
{
     /* Delete the RC_RX task temporarily to exit any blocking loops */
	 vTaskDelete(RC_RX_ID);

	 /* Recreate the RC_RX task to set the dron in hover mode */
	 xTaskCreate(xHandleRCRxTask,"RC_RX",200,NULL,2,&RC_RX_ID);

	 /* Wait 3 seconds for the drone to stabilize in hover */
	 vTaskDelay(pdMS_TO_TICKS(3000));
}

/**
 * @brief Disables non-essential peripherals to save power during failsafe.
 *        Turns off clocks for ADC, DMA, and USART.
 */
void disable_periph(void)
{
    /* Disable clock access to ADC1 to save power */
	RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;

    /* Disable clock access to DMA2 to save power */
	RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;

    /* Disable clock access to USART1 to save power */
	RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
}

