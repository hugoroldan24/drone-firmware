/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file spi.c																	   *
 * @brief															               *																			   *
 * This module configures SPI2 and its GPIO pins, and provides non-blocking	       *
 * transmit/receive functions integrated with FreeRTOS task notifications.		   *
 * Communication uses interrupts for TX and RX events, avoiding CPU busy-waiting.  *
 *																				   *
 * Pins used:																	   *
 * PB10: SCK2																 	   *
 * PB15: MOSI2																       *
 * PB14: MISO2																       *
 * PB8 : SS (software-controlled)											       *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f411xe.h"
#include "const.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

static void spi2_tx_callback(void);
static void spi2_rx_callback(void);
static void SPI_GPIO_Init(void);

static volatile uint8_t *tx_buffer;
static volatile uint32_t tx_size;
static volatile uint32_t tx_index;
static volatile uint8_t *rx_buffer;
static volatile uint32_t rx_size;
static volatile uint32_t rx_index;
static volatile uint32_t temp;

extern TaskHandle_t SENSORS_ID;
static volatile BaseType_t pxHigherPriorityTaskWoken;

/**
 * @brief Configures GPIO pins for SPI2 communication and chip select.
 */
static void SPI_GPIO_Init(void)
{
	/* Ensure CS (PB8) is high before configuring as output */
	GPIOB->ODR |= GPIO_ODR_OD8;

	/* Configure PB8 as output for CS */
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER8) | (0x1 << GPIO_MODER_MODER8_Pos);

	/* Configure PB10 (SCK), PB14 (MISO), PB15 (MOSI) as alternate function */
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER10) | (0x2 << GPIO_MODER_MODER10_Pos);
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER14) | (0x2 << GPIO_MODER_MODER14_Pos);
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER15) | (0x2 << GPIO_MODER_MODER15_Pos);

	/* Set alternate function AF5 (SPI2) for PB10, PB14, PB15 */
	GPIOB->AFR[1] = ((GPIOB->AFR[1])&~(GPIO_AFRH_AFSEL10)) | (0x5 << GPIO_AFRH_AFSEL10_Pos);
	GPIOB->AFR[1] = ((GPIOB->AFR[1])&~(GPIO_AFRH_AFSEL14)) | (0x5 << GPIO_AFRH_AFSEL14_Pos);
	GPIOB->AFR[1] = ((GPIOB->AFR[1])&~(GPIO_AFRH_AFSEL15)) | (0x5 << GPIO_AFRH_AFSEL15_Pos);
}

/**
 * @brief Initializes SPI2 peripheral with proper configuration (mode, prescaler, interrupts).
 */
void SPI_Init(void)
{
	/* Configure GPIOs for SPI2 */
	SPI_GPIO_Init();

	/* Enable clock for SPI2 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	/* Configure SPI speed: APB1 / 32 = 12.5 MHz */
	SPI2->CR1 = (SPI2->CR1 & ~SPI_CR1_BR) | (0x4 << SPI_CR1_BR_Pos);

	/* Configure SPI mode: CPOL=1, CPHA=1 (Mode 3) */
	SPI2->CR1 |= SPI_CR1_CPOL;
	SPI2->CR1 |= SPI_CR1_CPHA;

	/* 8-bit data frame */
	SPI2->CR1 &= ~SPI_CR1_DFF;

	/* Send MSB first */
	SPI2->CR1 &= ~SPI_CR1_LSBFIRST;

	/* Full-duplex mode */
	SPI2->CR1 &= ~SPI_CR1_RXONLY;

	/* MCU is master */
	SPI2->CR1 |= SPI_CR1_MSTR;

	/* Enable software slave management and set SSI */
	SPI2->CR1 |= SPI_CR1_SSM;
	SPI2->CR1 |= SPI_CR1_SSI;

	/* Enable error interrupt */
	SPI2->CR2 |= SPI_CR2_ERRIE;

	/* Enable NVIC interrupt for SPI2 */
	NVIC_EnableIRQ(SPI2_IRQn);

	/* Enable SPI2 peripheral */
	SPI2->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief Sends data over SPI2 using interrupts and task notifications (non-blocking).
 * @param data Pointer to transmit buffer.
 * @param size Number of bytes to send.
 */
void SPI_Transmit(uint8_t *data, uint32_t size)
{
	/* Store transmit buffer and size */
	tx_buffer = data;
	tx_size = size;
	tx_index = 0;

	/* Enable TXE interrupt to start transmission */
	SPI2->CR2 |= SPI_CR2_TXEIE;

	/* Block task until ISR signals completion */
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

	/* Wait until last byte has been shifted out */
    while(SPI2->SR & SPI_SR_BSY);

    /* Read DR to clear last dummy data */
    (void)SPI2->DR;
}

/**
 * @brief Receives data over SPI2 using interrupts and task notifications (non-blocking).
 * @param data Pointer to receive buffer.
 * @param size Number of bytes to receive.
 */
void SPI_Receive(uint8_t *data, uint32_t size)
{
	/* Store receive buffer and size */
	rx_buffer = data;
	rx_size = size;
	rx_index = 0;

	/* Clear possible residual data */
	(void)SPI2->DR;

	/* Enable RXNE interrupt */
	SPI2->CR2 |= SPI_CR2_RXNEIE;

	/* Write dummy data to trigger clock and reception */
	SPI2->DR = DUMMY;

	/* Block task until ISR signals completion */
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

	/* Wait until SPI bus is free */
	while(SPI2->SR & SPI_SR_BSY);
}

/**
 * @brief Handles SPI2 transmit interrupt, sending next byte or signaling completion.
 */
static void spi2_tx_callback(void)
{
	/* Send next byte if available */
	if(tx_index < tx_size)
	{
		SPI2->DR = tx_buffer[tx_index++];
	}
	else
	{
		/* Disable TXE interrupt once done */
		SPI2->CR2 &= ~SPI_CR2_TXEIE;

		/* Notify waiting task that transfer is complete */
		pxHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(SENSORS_ID,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}

/**
 * @brief Handles SPI2 receive interrupt, storing received byte or signaling completion.
 */
static void spi2_rx_callback(void)
{
	/* Store received byte */
	rx_buffer[rx_index++] = SPI2->DR;

	if(rx_index < rx_size)
	{
		/* Send dummy byte to keep clock running */
		SPI2->DR = DUMMY;
	}
	else
	{
		/* Disable RXNE interrupt once buffer is full */
		SPI2->CR2 &= ~SPI_CR2_RXNEIE;

		/* Notify waiting task that reception is complete */
		pxHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(SENSORS_ID,0,eNoAction,NULL);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}

/**
 * @brief Pulls CS (PB8) low to enable the SPI slave.
 */
void cs_enable(void){GPIOB->ODR &= ~GPIO_ODR_OD8;}

/**
 * @brief Pulls CS (PB8) high to disable the SPI slave.
 */
void cs_disable(void){GPIOB->ODR |= GPIO_ODR_OD8;}

/**
 * @brief Interrupt handler for SPI2 (transmit, receive, error).
 */
void SPI2_IRQHandler(void)
{
	uint32_t sr =  SPI2->SR;
	uint32_t cr2 = SPI2->CR2;

	/* Handle TXE interrupt */
	if((sr & SPI_SR_TXE) && (cr2 & SPI_CR2_TXEIE))
	{
		spi2_tx_callback();
	}

	/* Handle RXNE interrupt */
	if((sr & SPI_SR_RXNE) && (cr2 & SPI_CR2_RXNEIE))
	{
		spi2_rx_callback();
	}

	/* Handle overrun error */
	if(sr & SPI_SR_OVR)
	{
		/* Clear OVR flag by reading DR then SR */
		temp = SPI2->DR;
		temp = SPI2->SR;
	}
}

