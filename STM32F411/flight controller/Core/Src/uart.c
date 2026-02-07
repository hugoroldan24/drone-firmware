/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file uart.c																			   *
 * @brief																				   *
 * This file manages UART1 communication using DMA for minimal CPU intervention.		   *
 * Functions here configure the UART peripheral, set up DMA for both transmit			   *
 * and receive operations, and provide APIs for sending and receiving data.				   *
 * Double-buffering is used for DMA reception to safely handle concurrent access.		   *
 * Direct-to-task notifications from ISRs are used instead of semaphores for efficiency.   *
 *                                                                                         *
 * In reception, desabling the transfer complete interrupt does not stop or pause the DMA  *
 * from receiving bytes in the background. The DMA continues to store incoming bytes       *
 * sequentially, maintaining the correct order.                                            *
 *                                                                                         *
 * Enabling the ISR only provides a point where we can safely access the fully received    *
 * data. This ensures that when the interrupt occurs, the buffer already contains the      *
 * joystick position values in the expected order.                                         *
 *  																					   *
 * Pins used:																			   *
 * PA9:  TX																				   *
 * PA10: RX	                                                                               *
 * PC15: Connection between FC and receiver												   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <stdint.h>
#include "const.h"
#include "stm32f411xe.h"
#include "util.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"


static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static void dma2_receiver_init(uint32_t src, uint32_t dst1, uint32_t dst2, uint32_t len);
static void dma2_transmitter_init(uint32_t dst);
static void dma2_uart1_init(void);


static volatile uint32_t   tx_len;
static volatile uint8_t    dma_rx_buffer1[FRAME_LEN];
static volatile uint8_t    dma_rx_buffer2[FRAME_LEN];
static volatile uint8_t    dma_tx_buffer[TELEM_FRAME_SIZE];
static volatile uint8_t    available_buf;
static volatile BaseType_t pxHigherPriorityTaskWoken;
extern TaskHandle_t 	   RC_RX_ID, SAFETY_ID;
static uint32_t 		   timeout;


/**
 * @brief Configures the USART baud rate based on the peripheral clock and desired baud rate.
 * @param USARTx Pointer to the USART peripheral.
 * @param PeriphClk Peripheral clock frequency in Hz.
 * @param BaudRate Desired baud rate.
 */
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
    /* Calculate the USARTDIV value based on peripheral clock and desired baud rate */
	float usartdiv = (float)PeriphClk / (float)BaudRate;

    /* Split into mantissa and fraction */
	uint16_t mantissa = (uint16_t)usartdiv;
	uint8_t fraction = (uint8_t)((usartdiv - mantissa) * 16 + 0.5f);

    /* Configure the BRR register */
	USARTx->BRR = (mantissa << 4) | (fraction & 0x0F);
}


/**
 * @brief Initializes DMA2 Stream 5 for UART1 reception using double-buffered circular mode.
 * @param src Peripheral data register address (source).
 * @param dst1 First memory buffer address.
 * @param dst2 Second memory buffer address.
 * @param len Number of bytes to transfer.
 */
static void dma2_receiver_init(uint32_t src, uint32_t dst1, uint32_t dst2, uint32_t len)
{
    /* Disable the DMA stream before configuration */
	DMA2_Stream5->CR &= ~DMA_SxCR_EN;

    /* Wait until the stream is fully disabled */
	while(DMA2_Stream5->CR & DMA_SxCR_EN);

    /* Clear all interrupt flags */
	DMA2->HIFCR |= 0xF40;

    /* Set peripheral and memory addresses */
	DMA2_Stream5->PAR = src;
	DMA2_Stream5->M0AR = dst1;
	DMA2_Stream5->M1AR = dst2;

    /* Set number of data items to transfer */
	DMA2_Stream5->NDTR = len;

    /* Select channel 4 */
	DMA2_Stream5->CR = (DMA2_Stream5->CR & ~DMA_SxCR_CHSEL) | (0x4 << DMA_SxCR_CHSEL_Pos);

    /* Enable memory increment mode */
	DMA2_Stream5->CR |= DMA_SxCR_MINC;

    /* Set transfer direction: peripheral to memory */
	DMA2_Stream5->CR = (DMA2_Stream5->CR & ~DMA_SxCR_DIR) | (0x00 << DMA_SxCR_DIR_Pos);

    /* Enable double buffer and circular mode */
	DMA2_Stream5->CR |= DMA_SxCR_DBM | DMA_SxCR_CIRC;

    /* Disable FIFO and enable direct mode */
	DMA2_Stream5->FCR = 0;

    /* Enable DMA stream */
	DMA2_Stream5->CR |= DMA_SxCR_EN;

    /* Enable UART receiver DMA */
	USART1->CR3 |= USART_CR3_DMAR;

    /* Enable NVIC interrupt for DMA2 Stream5 */
	NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}


/**
 * @brief Initializes DMA2 Stream 7 for UART1 transmission.
 * @param dst Peripheral data register address (destination).
 */
static void dma2_transmitter_init(uint32_t dst)
{
    /* Enable DMA clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* Disable the stream before configuration */
	DMA2_Stream7->CR &= ~DMA_SxCR_EN;

    /* Wait until stream is disabled */
	while(DMA2_Stream7->CR & DMA_SxCR_EN);

    /* Set peripheral address (destination) */
	DMA2_Stream7->PAR = dst;

    /* Select channel 4 */
	DMA2_Stream7->CR = (DMA2_Stream5->CR & ~DMA_SxCR_CHSEL) | (0x4 << DMA_SxCR_CHSEL_Pos);

    /* Enable memory increment */
	DMA2_Stream7->CR |= DMA_SxCR_MINC;

    /* Set transfer direction: memory to peripheral */
	DMA2_Stream7->CR = (DMA2_Stream5->CR & ~DMA_SxCR_DIR) | (0x01 << DMA_SxCR_DIR_Pos);

    /* Disable FIFO and enable direct mode */
	DMA2_Stream7->FCR = 0;

    /* Enable UART transmitter DMA */
	USART1->CR3 |= USART_CR3_DMAT;

    /* Enable transfer complete interrupt */
	DMA2_Stream7->CR |= DMA_SxCR_TCIE;

    /* Enable NVIC interrupt */
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/**
 * @brief Initializes both DMA streams for UART1 (receiver and transmitter).
 */
static void dma2_uart1_init()
{
    /* Enable DMA2 clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* Initialize receiver and transmitter streams */
    dma2_receiver_init((uint32_t)&USART1->DR,(uint32_t)dma_rx_buffer1, (uint32_t)dma_rx_buffer2,FRAME_LEN);
    dma2_transmitter_init((uint32_t)&USART1->DR);
}

/**
 * @brief Configures UART1 pins, sets baud rate, enables transmitter and receiver, and initializes DMA streams.
 */
void uart1_txrx_init_dma()
{
	GPIOC->MODER |= (1U << 30U)  /*Set PC15 as output*/

    /* Configure PA9 and PA10 as alternate function for UART1 TX and RX */
	GPIOA->MODER |= (1U<<19);
	GPIOA->MODER &= ~(1U<<18);
	GPIOA->MODER |= (1U<<21);
	GPIOA->MODER &= ~(1U<<20);

    /* Set alternate function AF7 for UART1 */
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~GPIO_AFRH_AFSEL9)  | (7 << GPIO_AFRH_AFSEL9_Pos);
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~GPIO_AFRH_AFSEL10) | (7 << GPIO_AFRH_AFSEL10_Pos);

    /* Enable USART1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* Configure baud rate */
    uart_set_baudrate(USART1, APB2_CLK, BAUD);

    /* Enable transmitter and receiver */
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    /* Set 8-bit word length */
    USART1->CR1 &= ~USART_CR1_M;

    /* Enable USART */
    USART1->CR1 |= USART_CR1_UE;

    /* Initialize DMA streams for UART1 */
    dma2_uart1_init();
}


/**
 * @brief Starts a DMA transmission on UART1 with the given data buffer.
 * @param data Pointer to the data buffer.
 * @param len Number of bytes to transmit.
 */
void uart1_send_dma(uint8_t *data, uint32_t len)
{
	/*
	   Normally we should check that no DMA transmission is ongoing,
	   but since send_telemetry is called every 200 ms, the previous transfer is guaranteed to have finished.
	   The Stream7 is already disabled by the ISR, so we avoid waiting for the hardware to stop it manually.
	 */

    /* Clear any pending DMA interrupts */
	DMA2->HIFCR |= 0xF400000;

    /* Set memory address and number of bytes to transmit */
	DMA2_Stream7->M0AR = (uint32_t)data;
	DMA2_Stream7->NDTR = len;

    /* Enable DMA stream to start transmission */
	DMA2_Stream7->CR |= DMA_SxCR_EN;
}

/**
 * @brief Waits for DMA reception to complete, verifies checksum, and copies data to provided buffer.
 *        Triggers failsafe if timeout occurs.
 * @param buf Pointer to buffer where received data will be stored.
 * @param len Number of bytes to read (excluding checksum).
 * @return 1 if checksum is correct, 0 otherwise.
 */
uint8_t uart1_read_dma(uint8_t* buf, uint16_t len)
{
	uint8_t  expected_cs = 0;
	uint16_t sum = 0;
	uint8_t *active_buf;

    /* Clear pending DMA flags */
	DMA2->HIFCR |= DMA_HIFCR_CTCIF5;

    /* Enable transfer complete interrupt */
	DMA2_Stream5->CR |= DMA_SxCR_TCIE;

    /* Wait until DMA transfer completes or timeout occurs */
	/* If after 100 ms the interrupt is not triggered, we assume connection lost*/
	timeout =ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(100U));

    /* Trigger failsafe if timeout occurs (connection lost with the transmitter) */
	if(timeout == (uint32_t)pdFALSE){
		vTaskResume(SAFETY_ID);
	}

    /* Select active buffer */
	active_buf = (available_buf == 1) ? (uint8_t *) dma_rx_buffer1 : (uint8_t *) dma_rx_buffer2;

    /* Read expected checksum */
	expected_cs = active_buf[FRAME_LEN-1];

    /* Copy data excluding checksum */
	memcpy(buf, active_buf, FRAME_LEN-1);

    /* Calculate checksum */
	for(uint16_t i=0;i<len;i++) sum += buf[i];
	sum &= 0xFF;

    /* Compare checksum */
	return (expected_cs == sum) ? 1 : 0;
}

/**
 * @brief Handles DMA2 Stream 5 transfer complete interrupt, switches active buffer,
 *        and notifies the RC_RX task.
 */
static void DMA2_Stream5_callback()
{
	/* Disable transfer complete interrupt to ensure the UART buffer is not overwritten */
	DMA2_Stream5->CR &= ~DMA_SxCR_TCIE;

    /* Update available buffer according to CT bit */
	if(DMA2_Stream5->CR & DMA_SxCR_CT)		available_buf = 1U; /* Second buffer available */
	else									available_buf = 2U; /* First buffer available */

    /* Notify the RC_RX task from ISR */
	pxHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(RC_RX_ID,&pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken); 
}	


/**
 * @brief Handles DMA2 Stream 7 transfer complete interrupt and disables the stream.
 */
static void DMA2_Stream7_callback()
{
	/*
	   Disable the DMA stream after transmission.
	   Since send_telemetry is called infrequently (~200 ms), the hardware has enough time
	   to complete and disable the DMA on its own, so we avoid the blocking wait loop.
	*/
	DMA2_Stream7->CR &= ~DMA_SxCR_EN;
	GPIOC->ODR |= GPIO_ODR_OD15; /* Aquí quedaria decidir que pin usar, ojo que habrá que ponerlo como output*/
	/* Como el receptor necesita que mantengamos activo el pin minimo 62.5 ns, hacemos 7 operacions nop para llegar a 70 ns*/
	/* Como trabajamos a 100 MHz, cada ciclo son 10 ns -> 7 ciclos*/
	__asm volatile(
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
	);
    GPIOC->ODR &= ~GPIO_ODR_OD15;
}

 /**
  * @brief High-priority ISR for DMA2 Stream 5.
  *        Calls DMA2_Stream5_callback() if transfer complete.
  */
void DMA2_Stream5_IRQHandler(void)
{
    /* Check transfer complete flag */
	if(DMA2->HISR & DMA_HISR_TCIF5)
	{
		/* Clear flag and call callback */
		DMA2->HIFCR |= DMA_HIFCR_CTCIF5;
		DMA2_Stream5_callback();
	}
}

/**
 * @brief Low-priority ISR for DMA2 Stream 7.
 *        Calls DMA2_Stream7_callback() if transfer complete.
 */
void DMA2_Stream7_IRQHandler(void)
{
    /* Check transfer complete flag */
	if(DMA2->HISR & DMA_HISR_TCIF7)
	{
        /* Clear flag and call callback */
		DMA2->HIFCR |= DMA_HIFCR_CTCIF7;
		DMA2_Stream7_callback();
	}
}









