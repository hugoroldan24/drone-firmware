/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 

/***********************************************************************************************
 * spi.c
 *
 * Brief Description:
 * This module configures and manages the SPI peripheral on the ATmega328P in master mode.
 * It provides:
 *   - An ISR to handle “transaction complete” events and capture received bytes.
 *   - Initialization of SPI pins and control registers.
 *   - Blocking functions to send a byte and receive a byte via SPI.
 *
 * Public Functions:
 *   - void SPI_Init(void);
 *       Configures MOSI, SCK, and SS pins, enables SPI in master mode, sets clock phase/polarity,
 *       and enables the SPI interrupt.
 *
 *   - void SPI_Send_Data(uint8_t data);
 *       Sends one byte over SPI and waits (blocking) until the transfer complete ISR sets a flag.
 *
 *   - void SPI_Receive_Data(uint8_t dummy, uint8_t *obtainedData);
 *       Sends a dummy byte to generate clock pulses, waits for transfer complete, and returns
 *       the received byte via pointer.
 *
 * Interrupt Service Routines:
 *   - ISR(SPI_STC_vect):
 *       Triggered when an SPI transfer completes. Reads SPDR into a buffer and signals the main
 *       code that transmission/reception is done.
 *
 ***********************************************************************************************/
 
 
#include "common.h"
#include <avr/interrupt.h>


volatile int8_t spi_tx_done = 0;
volatile uint8_t receivedData;


/**
 * @brief  SPI Transfer Complete interrupt service routine.
 *         Captures the received byte from the SPI data register (SPDR) into a buffer and
 *         sets a flag so that blocking transfer functions know the transfer is finished.
 */
ISR(SPI_STC_vect)
{
  receivedData = SPDR;	 /*We get the data from the reception buffer*/
  spi_tx_done = 1;
}


/**
 * @brief  Initialize the SPI peripheral in master mode.
 *         - Sets MOSI, SCK, and SS pins as outputs.
 *         - Enables SPI, sets the ATmega328P as master.
 *         - Configures clock polarity/phase to 0 (mode 0).
 *         - Enables the SPI interrupt for transfer-complete events.
 *         - Pulls SS_PIN high (inactive state) to avoid accidental slave selection.
 */
void SPI_Init()
{	
  DDRB = DDRB | ((1<<DD_MOSI) | (1<<DD_SS_SLAVE) | (1<<DD_CLK));      /* Set MOSI, SCK, and SS_SLAVE as outputs. We set SS_SLAVE as output so that the MCU does not return to slave mode */
  DDRD |= (1<<DD_SS);
  SPCR =  SPCR | ((1<<MSTR) | (1<<SPE));                              /* Configure as SPI master and enable SPI */
  SPCR = SPCR & ((~(1<<CPOL)) & (~(1<<CPHA)));                        /* Clock Polarity = 0, Phase = 0 (SPI mode 0) */
  SPCR |= (1<<SPIE);						      /* Enable SPI Transfer Complete interrupt */
                                                          
  PORTD |= (1 << SS_PIN);				 	      /* Set SS_PIN in pull-up mode */
  /* We don’t need to adjust prescaler bits since default gives 4 MHz SPI clock at 16 MHz CPU */
}


/**
 * @brief  Send a single byte over SPI in blocking mode.
 *         - Writes data to SPDR to start transfer.
 *         - Waits for the SPI_STC_vect ISR to set spi_tx_done = 1.
 *         - Clears the flag before returning.
 * @param  data  Byte to transmit.
 */
void SPI_Send_Data(uint8_t data)
{
  SPDR = data;           /*Send data to Slave*/
  while(!spi_tx_done);	 /*Wait until transaccion is completed*/
  spi_tx_done = 0;                           		     
}


/**
 * @brief  Receive a single byte over SPI in blocking mode.
 *         - Sends a dummy byte to generate clock pulses.
 *         - Waits for the SPI_STC_vect ISR to set spi_tx_done = 1.
 *         - Reads the received byte from the global buffer into *obtainedData.
 * @param  dummy         Dummy byte to send (often 0xFF or 0x00).
 * @param  obtainedData  Pointer to a variable where the received byte will be stored.
 */
void SPI_Receive_Data(uint8_t dummy, uint8_t *obtainedData)
{
  SPDR = dummy;                       /*Send dummy data to Slave*/
  while(!spi_tx_done);		      /* Wait until transaction is completed */
  spi_tx_done = 0;        			  
  *obtainedData = receivedData;       /* Return the received byte */                  
}
