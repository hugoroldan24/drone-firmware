/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */


/***********************************************************************************************
 * radio.c
 *
 * Brief Description:
 * This module provides low-level routines to configure and operate the nRF24L01+ RF transceiver
 * on the ATmega328P in transmitter mode. It includes:
 *   - Functions to write/read RF registers, send commands, set pipe addresses.
 *   - Initialization of the nRF24L01+ as a transmitter.
 *   - A function to send payloads without ACK. Since we don't need to receive any information from
       the receiver and the system is paquet-loss resilient, we can avoid the use of ACKs.
 *
 * Functions:
 *   - void writeRegister(uint8_t writeCommand, uint8_t conf);
 *       Writes a single byte configuration value to the specified nRF24L01+ register.
 *
 *   - uint8_t readRegister(uint8_t reg);
 *       Reads and returns the value of the specified nRF24L01+ register.
 *
 *   - void sendCommand(uint8_t command);
 *       Sends a single-byte command to the nRF24L01+ (e.g., FLUSH_TX).
 *
 *   - void writeAddress(uint8_t pipe, uint8_t *addr, uint8_t size);
 *       Writes an address (multiple bytes) to the given RX or TX address pipe.
 *
 *   - void RF_Transmitter_Init(void);
 *       Configures the nRF24L01+ as a transmitter: channel, data rate, address width,
 *       feature activation, config register, and flushes TX FIFO.
 *
 *   - void sendPaquet(uint8_t *data, uint8_t size);
 *       Sends a payload of specified size without auto-ACK. Asserts CE for the required pulse.
 *
 ***********************************************************************************************/
 
 
#include "spi.h"
#include "common.h"
#include <avr/interrupt.h>

volatile uint8_t received_telem = 0;

ISR(INT0_vect)
{
   received_telem = 1;
}

/**
 * @brief  Write a configuration value to a specific nRF24L01+ register.
 * @param  writeCommand  The SPI write command register address.
 * @param  conf          The configuration byte to write into the register.
 */
void writeRegister(uint8_t writeCommand,uint8_t conf)
{
   PORTD &= ~(1 << SS_PIN);            	
   SPI_Send_Data(writeCommand);
   SPI_Send_Data(conf); 			
   PORTD |= (1 << SS_PIN);  
}


/**
 * @brief  Read and return a single-byte value from the specified nRF24L01+ register.
 * @param  reg  The SPI command/register address (R_REGISTER | register).
 * @return uint8_t data The byte read from that register.
 */
uint8_t readRegister(uint8_t reg)
{
   uint8_t data;
   PORTD &= ~(1 << SS_PIN);            	
   SPI_Send_Data(reg);
   SPI_Receive_Data(NOP,&data); 			
   PORTD |= (1 << SS_PIN);
   return data;   
}


/**
 * @brief  Send a single-byte command to the nRF24L01+ module.
 * @param  command  The command you want to send to the module (e.g., FLUSH_TX, NOP).
 */
void sendCommand(uint8_t command)
{
   PORTD &= ~(1 << SS_PIN);            
   SPI_Send_Data(command); 			
   PORTD |= (1 << SS_PIN);
}


/**
 * @brief  Write a multi-byte address into the specified pipe register.
 * @param  pipe   The SPI command for the target address register (e.g., W_TX_ADDR).
 * @param  addr   Pointer to the array containing the address bytes.
 * @param  size   Number of address bytes to send.
 */
void writeAddress(uint8_t pipe,uint8_t *addr,uint8_t size)
{
  PORTD &= ~(1 << SS_PIN);		
  SPI_Send_Data(pipe);
  for(int8_t l=0;l<size;l++){
     SPI_Send_Data(addr[l]);
  }
  PORTD |= (1 << SS_PIN); 
}


/**
 * @brief  Initialize the nRF24L01+ as a wireless transmitter.
 *         Configures CE pin, sets channel, data rate, address width,
 *         disables retransmit/auto-ack, activates TX_NO_ACK feature, and
 *         programs the CONFIG register for TX mode. Flushes TX FIFO.
 */
void RF_Transmitter_Init()
{
  uint8_t tx_address[] = {0xE7,0xE7,0xE7,0xE7,0xE7};
   
  DDRD  &= ~(1 << DD_INT0);                   /* INT0 as input */
  PORTD |= (1 << INT0_PIN);		           /* Enable pull-up on PD2 */
  EICRA |= (1 << ISC01);                    /* Trigger INT0 on falling edge */ 
  EIFR  |= (1 << INTF0);			           /* Clear any pending INT0 flag */
  EIMSK |= (1 << INT0);  		              /* Unmask INT0 interrupt */

  DDRB |= (1 << DD_CE);		          		/* Configure CE pin as output */
  PORTB &= ~(1 << CE_PIN); 		  		       /* Ensure CE = 0 */
  
  _delay_us(10300); 					       /* Wait ~10.3 ms for power-down stabilization */
  writeRegister(W_RF_CH,0x04);         	        	/* Set RF channel to 2.404 GHz (avoid Wi-Fi) */          	    
  writeRegister(W_RF_SETUP,0x0F);			       /* 2 Mbps data rate, 0 dBm, enable LNA gain */
  writeRegister(W_SETUP_AW,0x03);			       /* Address width = 5 bytes */
  /* Ahora no tenemos porque desactivar auto-retransmit, ya que si el No_ACK esta activo, no importa que esté activado*/
  /* Pero para enviar paquetes y recibir la payload, debemos tenerlo activado para pponer entrar en modo RX*/
  //writeRegister(W_SETUP_RETR,0x00);	       	/* Disable auto-retransmit */
  writeRegister(W_STATUS,0x3E);				/* Clear TX_DS and MAX_RT flags */
  //writeRegister(W_EN_AA,0x00);				/* Disable auto-acknowledgment on all pipes */
  writeAddress(W_TX_ADDR,tx_address,ADDRESS_WIDTH); 	/* Set TX address */   
  writeRegister(ACTIVATE,ACTIVATION_KEY); 	       /* Activate features (enable TX_NO_ACK) */
  writeRegister(W_FEATURE,0x01);	 		       /* Enable TX payload no-ACK feature */

  writeRegister(W_CONFIG,0x3A); 	 		       /* PWR_UP=1, PRIM_RX=0, enable CRC, mask TX and MAX_RT interrupts */

  _delay_us(1500);                           	       /* Power-up delay (~1.5 ms) */  
 						              /* After configuration, module is in Standby-I until CE=1 */
  sendCommand(FLUSH_TX);                      		/* Flush TX FIFO */
}
  
  				  
/**
 * @brief  Send a data payload to the nRF24L01+ with W_TX_PAYLOAD_NOACK command.
 *         Does not wait for auto-ACK. Pulses CE for ~15 µs to transmit.
 * @param  joystick Union that contains both axis joystick payload
 *         no_ack   Flag used to determine if we want to paquet ACK or not
 */
void sendPaquet(JoystickData joystick,uint8_t no_ack)
{
   PORTD &= ~(1 << SS_PIN);        /* Chip Select ON */
   if(no_ack){
         SPI_Send_Data(W_TX_PAYLOAD_NO_ACK);  /* Write payload in TX FIFO with NO_ACK flag set */   	         	       
   }
   else{
         SPI_Send_Data(W_TX_PAYLOAD);  /* Write payload in TX FIFO with NO_ACK flag unset */   	         	       

   }
   SPI_Send_Data(joystick.x1_axis);	    /* Send joystick x1_axis data */
   SPI_Send_Data(joystick.y1_axis);	    /* Send joystick y1_axis data */
   SPI_Send_Data(joystick.x2_axis);	    /* Send joystick x2_axis data */
   SPI_Send_Data(joystick.y2_axis);	    /* Send joystick y2_axis data */
   
   PORTD |= (1 << SS_PIN); 	       /* Chip Select OFF */ 
   
   PORTB |= (1 << CE_PIN);         /* Set CE HIGH to transmit */
   _delay_us(15);			           /* 15 µs CE pulse to trigger TX */
   PORTB &= ~(1 << CE_PIN);        /* Set CE LOW to return to Standby-I after packet is transmitted */               	 	
}  


 /**
 * @brief  Read two bytes from the nRF24L01+ RX FIFO.
 *         Issues the R_RX_PAYLOAD command and reads two successive bytes into the
 *         provided pointers. Chip Select (CSN) is asserted low for the transaction.
 * @param  *joystick  Pointer to the struct where we will save the read bytes.
 */
 void get_Telem_Data(uint8_t* data, uint8_t len)
 {
   uint8_t i;
   PORTD &= ~(1 << SS_PIN);        	         /* Pull CSN low to begin SPI transaction */
   SPI_Send_Data(R_RX_PAYLOAD);    	         /* Send command to read RX FIFO */	
   for(i=0;i<len;i++){
      SPI_Receive_Data(NOP,&data[i]);
   }						  
   PORTD |= (1 << SS_PIN);         	         /* Release CSN */
 }


