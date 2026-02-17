/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file i2c.c																				   *
 *																							   *
 *@brief																					   *
 *Interrupt-driven I2C1 master driver that implements multi-byte read/write transactions via   *
 *a small state machine. Configures PB6/PB7 for I2C AF, sets Fast-mode timing (400 kHz),	   *
 *and handles START/ADDR/TXE/RXNE/BTF events in I2C1_EV_IRQHandler. The driver exposes		   *
 *i2c1_perform_action() to start a transaction; callbacks (start, address_sent, DR_empty,	   *
 *RX_ready, STOP_TX) progress the state machine and transfer data via shared volatile buffers  *
 *and flags.																		 		   *
 *																							   *
 *Pins used:																				   *
 *PB6: SCL																					   *
 *PB7: SDA																			   		   *																					   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f411xe.h"
#include <stdint.h>
#include "const.h"
#include "FreeRTOS.h"
#include "task.h"

static uint8_t set_rise_time(uint8_t tpclk);
static uint16_t set_CCR(uint32_t desired_period);
static void address_sent_callback(void);
static void DR_empty_callback(void);
static void RX_ready_callback(void);
static void start_callback(void);
static void STOP_TX_callback(void);
static void init_variables(void);

static volatile uint8_t slave_addr;            /* Slave 7-bit address for the current I2C transaction */
static volatile uint8_t* i2c_data;             /* Pointer to TX/RX data buffer used during the transfer */
static volatile uint8_t i2c_action;            /* Operation type/code (e.g., READ or WRITE) */
static volatile uint8_t send_data_flag;        /* Indicates driver should send next byte (used by TXE handler) */
static volatile uint8_t starting_register;     /* Register address sent first for read/write operations */
static volatile uint8_t len;                   /* Remaining byte count for the ongoing transfer */
static volatile I2C_State_t i2c_state;         /* Current state of the I2C state machine */
static volatile int8_t register_address_sent;  /* Flag (0/1) whether the register address was already sent */
static volatile int8_t RW = I2C_WRITE;         /* Read/Write indicator: I2C_READ or I2C_WRITE (defaults to WRITE) */
extern TaskHandle_t IMU_ID;
static volatile BaseType_t pxHigherPriorityTaskWoken;

/**
 * @brief  Compute TRISE value for Fast-mode (Fm) I2C based on tpclk.
 *         Returns the TRISE register value calculated from the clock period
 *         programmed in FREQ (tpclk, in ns) and the maximum allowed rise time.
 */
static uint8_t set_rise_time(uint8_t tpclk)
{
   return (uint8_t)((MAX_FM / tpclk) + 1);
}

/**
 * @brief  Compute CCR (clock control) value to achieve a desired SCL period.
 *         Converts a desired period in nanoseconds into the CCR value used by
 *         the I2C peripheral (uses APB1_PERIOD_NS and the timer scaling factor).
 */
static uint16_t set_CCR(uint32_t desired_period) /* ns */
{
   return (uint16_t)(desired_period/(25U*APB1_PERIOD_NS));
}

/**
 * @brief  Initialize I2C1 peripheral: configures PB6/PB7 for I2C alternate function,
 *         sets Fast-mode timing (400 kHz), enables the I2C1 peripheral and its
 *         event/buffer interrupts.
 */
void I2C1_Init()
{
	/* Set PB6 pin to Alternate Function Mode */
	GPIOB->MODER |= (1U<<13);
	GPIOB->MODER &= ~(1U<<12);

	/* Set PB7 pin to Alternate Function Mode */
	GPIOB->MODER |= (1U<<15);
	GPIOB->MODER &= ~(1U<<14);

	/* Select AF4 for PB6 (SCL) in AFRL */
	GPIOB->AFR[0] = ((GPIOB->AFR[0])&~(GPIO_AFRL_AFSEL6)) | (0x4 << GPIO_AFRL_AFSEL6_Pos);

	/* Select AF4 for PB7 (SDA) in AFRL */
	GPIOB->AFR[0] = ((GPIOB->AFR[0])&~(GPIO_AFRL_AFSEL7)) | (0x4 << GPIO_AFRL_AFSEL7_Pos);

	/* Configure PB6 and PB7 as open-drain output type (required for I2C) */
	GPIOB->OTYPER |= GPIO_OTYPER_OT6;
	GPIOB->OTYPER |= GPIO_OTYPER_OT7;

	/* Enable internal pull-up for PB6 and PB7		  	 */
	GPIOB->PUPDR = ((GPIOB->PUPDR)&~(GPIO_PUPDR_PUPDR6)) | (0x1 << GPIO_PUPDR_PUPD6_Pos);
	GPIOB->PUPDR = ((GPIOB->PUPDR)&~(GPIO_PUPDR_PUPDR7)) | (0x1 << GPIO_PUPDR_PUPD7_Pos);

	/* Enable peripheral clock for I2C1 on APB1 */
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	/* Put I2C1 peripheral into software reset */
	I2C1->CR1 |= I2C_CR1_SWRST;

	/* Release I2C1 from software reset */
	I2C1->CR1 &= ~I2C_CR1_SWRST;

	/* Set I2C peripheral frequency (FREQ) to APB1 clock in MHz */
	I2C1->CR2 = ((I2C1->CR2)&~(I2C_CR2_FREQ)) | (APB1_CLK_MHZ << I2C_CR2_FREQ_Pos);

	/* Select Fast-mode (Fm) for I2C */
	I2C1->CCR |= I2C_CCR_FS;

	/* Set duty cycle (DUTY = 1 => 16/9) to meet Fast-mode timing */
	I2C1->CCR |= I2C_CCR_DUTY;

	/* Compute and program CCR to achieve the desired SCL period (1/400kHz) */
	uint16_t CCR = set_CCR((uint32_t)DESIRED_T);
	I2C1->CCR = ((I2C1->CCR)&~(I2C_CCR_CCR)) | (CCR << I2C_CCR_CCR_Pos);

	/* Compute and program TRISE according to Fast-mode rise time constraints */
	uint8_t rise_time = set_rise_time(APB1_PERIOD_NS);
	I2C1->TRISE = ((I2C1->TRISE)&~(I2C_TRISE_TRISE)) | (rise_time << I2C_TRISE_TRISE_Pos);

	/* Enable the I2C peripheral */
	I2C1->CR1 |= I2C_CR1_PE;

	/* Enable event and buffer interrupts in the I2C peripheral */
	I2C1->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

	/* Enable I2C1 event interrupt in the NVIC */
	NVIC_EnableIRQ(I2C1_EV_IRQn);
}

/**
 * @brief  Start an I2C master transaction handled by the driver's ISR-driven state machine.
 *         Prepares the internal buffers/state, issues a START condition and then blocks
 *         until the requested transfer completes (driver clears `len` to 0).
 *
 * @param  action  Operation code indicating READ or WRITE transaction type.
 * @param  addr    7-bit I2C slave address to communicate with.
 * @param  maddr   Register/memory address inside the slave where the transfer begins.
 * @param  n       Number of bytes to transfer (TX bytes for write, RX bytes for read).
 * @param  data    Pointer to data buffer: source for writes or destination for reads.
 */
void i2c1_perform_action(uint8_t action, uint8_t addr, uint8_t maddr, int n, uint8_t* data)
{
	/* Wait until the I2C bus/peripheral is not busy from a previous transfer */
	while (I2C1->SR2 & I2C_SR2_BUSY);
	
	/* Load transaction parameters into driver-managed volatile variables */
	slave_addr = addr;
	starting_register = maddr;
	i2c_data = data;
	len = n;
	i2c_action = action;

	/* Initialize internal state flags and state machine */
	init_variables();

	/* Issue START condition to begin the I2C transaction — ISR will take over */
	I2C1->CR1 |= I2C_CR1_START;

	/* Block here until the ISR-driven transfer completes and notifies */
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
}

/**
 * @brief  I2C1 Event Interrupt Handler.
 *         This ISR drives the I2C master state machine by checking SR1 event flags
 *         and invoking the appropriate callback function depending on the current state.
 *         It handles start condition, address transmission, data TX/RX, and stop condition.
 *
 * @note   Triggered automatically by hardware on I2C1 events such as:
 *         - SB (Start Bit sent, EV5)
 *         - ADDR (Address sent/matched, EV6)
 *         - TXE (Data register empty, EV8)
 *         - RXNE (Data register not empty, EV7)
 *         - BTF (Byte transfer finished, EV8_2)
 */
void I2C1_EV_IRQHandler()
{
	(void)I2C1->SR1;	/* Dummy read to clear flags where required */

	if((I2C1->SR1 & I2C_SR1_SB)    && i2c_state == EV5)	    start_callback();
	if((I2C1->SR1 & I2C_SR1_ADDR)  && i2c_state == EV6)     address_sent_callback();
	if((I2C1->SR1 & I2C_SR1_TXE)   && i2c_state == EV8)     DR_empty_callback();
	if((I2C1->SR1 & I2C_SR1_RXNE)  && i2c_state == EV7)     RX_ready_callback();
	if((I2C1->SR1 & I2C_SR1_BTF)   && i2c_state == EV8_2)   STOP_TX_callback();
}

/**
 * @brief  Start condition callback.
 *         This function is called when the Start Bit (SB) flag is set (EV5).
 *         It writes the slave address along with the read/write bit into the data register,
 *         which clears the SB flag and transitions the I2C state machine to EV6.
 */
static void start_callback()
{
	I2C1->DR = (slave_addr << 1) | RW; /* Send slave address with R/W bit, clears SB flag */
	i2c_state = EV6;				   /* Move to next state (waiting for address event) */
}


/**
 * @brief  Address sent callback.
 *         Triggered when the ADDR flag is set (EV6), meaning the slave address has been
 *         successfully transmitted and acknowledged.
 *
 *         - In WRITE mode: clears ADDR, prevents spurious TxE interrupts, and writes the
 *           target register address before sending data.
 *
 *         - In READ mode:
 *              · If the register address has not yet been sent, it sends it and prepares for
 *                a repeated START (transition to EV8).
 *              · If the register address has already been sent, it prepares the ACK/NACK logic:
 *                   • When only 1 byte is expected, ACK is disabled and STOP is scheduled.
 *                   • Otherwise, ACK is enabled to allow reception of multiple bytes.
 */
static void address_sent_callback()
{
	if(i2c_action == WRITE){
		i2c_state = EV8;
		(void)I2C1->SR2; 			 		  /* Clear ADDR flag */
		NVIC_ClearPendingIRQ(I2C1_EV_IRQn);   /* Avoid spurious TxE interrupt */
		I2C1->DR = starting_register; 		  /* Send target register address */
		send_data_flag = 1;
	}
	else{
		if(register_address_sent){
			i2c_state = EV7;
			if(len == 1){
				I2C1->CR1 &= ~I2C_CR1_ACK;  /* Disable ACK to send NACK after reception */
				(void)I2C1->SR2; 			/* Clear ADDR flag */
				I2C1->CR1 |= I2C_CR1_STOP;	/* Prepare STOP after receiving the single byte */
			}
			else{
				I2C1->CR1 |= I2C_CR1_ACK;  /* Enable ACK for multiple-byte reception */
				(void)I2C1->SR2; 		   /* Clear ADDR flag */
			}
		}
		else{
			(void)I2C1->SR2; 			 		 /* Clear ADDR flag */
			I2C1->DR = starting_register;		 /* Send register address to read from */
			register_address_sent = 1;
			i2c_state = EV8;   					 /* Transition to EV8 to handle TxE interrupt */
		}
	}
}

/**
 * @brief  Data register empty callback (EV8).
 *         Triggered when the TxE (Transmit Data Register Empty) flag is set,
 *         meaning the data register is ready for the next byte.
 *
 *         - In WRITE mode: sends the next data byte from the buffer.
 *           When the last byte is reached, transitions to EV8_2
 *           (waiting for BTF before STOP).
 *
 *         - In READ mode: this state is only reached before performing
 *           a repeated START. The mode is switched to READ, the state
 *           machine goes back to EV5, and a new START condition is issued.
 */
static void DR_empty_callback()
{
	if(i2c_action == WRITE){
		if(send_data_flag){
			I2C1->DR = *i2c_data++;
			if(--len == 0){				 /* Last byte sent */
				i2c_state = EV8_2;
			}
		}
	}
	/* In READ mode, reached only before repeated START */
	else{
		RW = I2C_READ;
		i2c_state = EV5;
		I2C1->CR1 |= I2C_CR1_START;	 /* Issue repeated START condition */
	}
}

/**
 * @brief  Receive buffer not empty callback (EV7).
 *         Triggered when the RXNE (Receive Buffer Not Empty) flag is set,
 *         meaning a byte has been received and is ready to be read.
 *
 *         - Reads the incoming byte from the data register into the buffer.
 *         - If the next byte is the last one (len == 1 after decrement),
 *           disables ACK to generate a NACK and issues a STOP condition
 *           to properly end the communication.
 */
static void RX_ready_callback()
{
	(*i2c_data++) = I2C1->DR;	   /* Store received byte */
	if(--len == 1){  			   /* If about to receive the last byte */
	   I2C1->CR1 &= ~I2C_CR1_ACK;  /* Disable ACK so the last byte is NACKed */
	   I2C1->CR1 |= I2C_CR1_STOP;  /* Issue STOP condition */
	}
	if(len == 0) /* If you already received the last byte, notify */
	{
		pxHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(IMU_ID,&pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}

/**
 * @brief  Stop condition callback (EV8_2).
 *         Triggered when the BTF (Byte Transfer Finished) flag is set
 *         after the last byte has been transmitted in a write operation.
 *
 *         - Issues a STOP condition to release the I2C bus.
 *         - Disables I2C event and buffer interrupts to prevent spurious
 *           ISR calls after STOP (observed behavior during debugging).
 */
static void STOP_TX_callback()
{
	I2C1->CR1 |= I2C_CR1_STOP;	 /* Generate STOP condition */
	/* Disable I2C interrupts to avoid unwanted ISR triggers after STOP */
	I2C1->CR2  &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

	/* Signal that I2C transaction completed */
	pxHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(IMU_ID,&pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

/**
 * @brief  Initializes I2C state machine variables before a new transaction.
 */
static void init_variables()
{
	send_data_flag = 0;
	register_address_sent = 0;
	RW = I2C_WRITE;
	i2c_state = EV5;
}




