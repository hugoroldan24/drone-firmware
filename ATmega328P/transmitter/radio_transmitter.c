/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */


/***********************************************************************************************
 * radio_transmitter.c
 *
 * Brief Description:
 * This module provides low-level routines to configure and operate the nRF24L01+ RF transceiver
 * on the ATmega328P in transmitter (PTX) mode. It handles IRQ notification (INT0) for incoming
 * telemetry ACK payloads, supports both NOACK and ACK transmission modes for joystick frames,
 * and reads telemetry payloads from the RX FIFO when received via ACKs.
 *
 * The transmitter sends joystick data on a fixed address without ACK for normal operation
 * (loss-resilient) but can request ACKs to receive telemetry back from the PRX.
 *
 * Functions:
 *   - writeRegister
 *   - readRegister
 *   - sendCommand
 *   - writeAddress
 *   - RF_Transmitter_Init
 *   - sendPaquet
 *   - get_Telem_Data
 *
 * Interrupt Service Routines:
 *   - ISR(INT0_vect)
 *
 ***********************************************************************************************/
 
 
#include "spi.h"
#include "common.h"
#include <avr/interrupt.h>

volatile uint8_t received_telem = 0;


/**
 * @brief  INT0 (IRQ) interrupt service routine for nRF24L01+ transmitter.
 *         Triggered on a falling edge when the RF module asserts IRQ indicating a
 *         RX data ready (ACK payload received).
 *         Sets received_telem flag so the aplication layer can read the RX FIFO.
 *         The IRQ can be triggered too by TX_DS, but it is disabled.
 */
ISR(INT0_vect)
{
   received_telem = 1;
}


/**
 * @brief  Write a 1-byte value into an nRF24L01+ register over SPI.
 * @param  writeCommand  SPI command (W_REGISTER | reg).
 * @param  conf          Byte value to write.
 */
void writeRegister(uint8_t writeCommand,uint8_t conf)
{
   PORTD &= ~(1 << SS_PIN);            	
   SPI_Send_Data(writeCommand);
   SPI_Send_Data(conf); 			
   PORTD |= (1 << SS_PIN);  
}


/**
 * @brief  Read and return a 1-byte value from an nRF24L01+ register over SPI.
 * @param  reg  SPI command (R_REGISTER | reg).
 * @return Register value read.
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
 * @brief  Send a single-byte command to the nRF24L01+.
 * @param  command  Command byte (e.g., FLUSH_RX, FLUSH_TX).
 */
void sendCommand(uint8_t command)
{
   PORTD &= ~(1 << SS_PIN);            
   SPI_Send_Data(command); 			
   PORTD |= (1 << SS_PIN);
}


/**
 * @brief  Write a multi-byte address to an nRF24L01+ address register (RX pipe or TX).
 * @param  pipe  Write command for the target register (e.g., W_RX_ADDR_P0, W_TX_ADDR).
 * @param  addr  Pointer to address bytes (LSB first as sent over SPI).
 * @param  size  Number of bytes to write.
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
 * @brief  Initialize the nRF24L01+ as a transmitter (PTX).
 *         Sets up IRQ pin on INT0, configures RF channel/bitrate/address width,
 *         activates special features (ACTIVATE + 0x73) to enable EN_NO_ACK_TX,
 *         programs CONFIG for PTX mode, and flushes TX FIFO.
 *
 *         Note: With NOACK enabled, joystick packets are fire-and-forget; ACK mode
 *         is used selectively to receive telemetry in response.
 */
void RF_Transmitter_Init()
{
  uint8_t tx_address[] = {0xE7,0xE7,0xE7,0xE7,0xE7};
  uint8_t rx_pipe0_address[] = {0xE7,0xE7,0xE7,0xE7,0xE7};

  /* --- Configure INT0 (PD2) as the nRF24L01+ IRQ input (active low) --- */
  DDRD  &= ~(1 << DD_INT0);    /* INT0 as input */
  PORTD |= (1 << INT0_PIN);    /* Enable pull-up on PD2 */
  EICRA |= (1 << ISC01);       /* Trigger INT0 on falling edge */ 
  EIFR  |= (1 << INTF0);	    /* Clear any pending INT0 flag */
  EIMSK |= (1 << INT0);  		 /* Unmask INT0 interrupt */

  /* --- CE pin: controlled by MCU for TX pulses --- */
  DDRB |= (1 << DD_CE);		        /* CE as output */
  PORTB &= ~(1 << CE_PIN); 		  /* Ensure CE = 0 */
  
  _delay_us(10300); 					       /* Wait ~10.3 ms for power-down stabilization */

  /* --- Basic RF setup --- */
  writeRegister(W_RF_CH,0x04);          /* Set RF channel to 2.404 GHz (avoid Wi-Fi) */          	    
  writeRegister(W_RF_SETUP,0x0F);		 /* 2 Mbps data rate, 0 dBm, enable LNA gain */
  writeRegister(W_SETUP_AW,0x03);		 /* Address width = 5 bytes */
  writeRegister(W_STATUS,0x7E);			 /* Clear TX_DS and MAX_RT flags */

  /* Ahora no tenemos porque desactivar auto-retransmit, ya que si el No_ACK esta activo, no importa que esté activado*/
  /* Pero para enviar paquetes y recibir la payload, debemos tenerlo activado para pponer entrar en modo RX*/
  //writeRegister(W_SETUP_RETR,0x00);	       	/* Disable auto-retransmit */

  writeRegister(W_EN_AA,0x01);				/* Disable auto-acknowledgment on all pipes */

  writeRegister(W_EN_RXADDR,0x01);	    		 /* Enable only pipe 0 */

  writeRegister(W_STATUS,0x7E);          	 /* Clear RX_DS flag and all the flags */  

  /* TX address (must match receiver pipe 0). */
  writeAddress(W_TX_ADDR,tx_address,ADDRESS_WIDTH); 
  
  /* Set pipe 0 address (to receive ACKs) */
  writeAddress(W_RX_ADDR_P0,rx_pipe0_address,ADDRESS_WIDTH);      

  /* --- Enable special features for NOACK transmission --- */
  writeRegister(ACTIVATE,ACTIVATION_KEY); 	                   /* Activate features (enable TX_NO_ACK) */
  writeRegister(W_FEATURE,(1 << EN_DYN_ACK_BIT) | (1 << EN_DPL_BIT) |
                          (1 << EN_ACK_PAY_BIT));	 /* EN_NO_ACK_TX: send payloads without ACK and Dynamic Payload*/

  writeRegister(W_DYNPD,(1 << DPL_P0_BIT)); /* Enable dyn. payload length data pipe 0. (required when PRX uses DPL)*/

  /* Auto retransmission is set by default */

  /* --- Power up in PTX mode --- */
  writeRegister(W_CONFIG,0x3A); 	 		       /* PWR_UP=1, PRIM_RX=0, enable CRC, mask TX and MAX_RT interrupts */

  _delay_us(1500);                            /* Power-up to Standby-I */  
 	
  /* After configuration, module is in Standby-I until CE=1 */
  sendCommand(FLUSH_TX);                       /* Clear TX FIFO */
  sendCommand(FLUSH_RX); 
}
  
  				  
/**
 * @brief  Transmit a joystick payload using W_TX_PAYLOAD[_NOACK] command.
 *         Loads 4-axis data into TX FIFO, pulses CE for transmission.
 *
 * @param  joystick  Joystick data struct with 4 axes bytes.
 * @param  no_ack    1 = use NOACK (fire-and-forget); 0 = request ACK (to receive telemetry).
 */
void sendPaquet(JoystickData joystick,uint8_t no_ack)
{
   PORTD &= ~(1 << SS_PIN);               /* Chip Select ON */
   if(no_ack)
   {
      SPI_Send_Data(W_TX_PAYLOAD_NO_ACK); /* No ACK requested */   	         	       
   }
   else
   {
      SPI_Send_Data(W_TX_PAYLOAD);        /* ACK requested (telemetry expected) */  	         	       
   }

   /* Load 4-byte joystick payload. */
   SPI_Send_Data(joystick.x1_axis);	    
   SPI_Send_Data(joystick.y1_axis);	    
   SPI_Send_Data(joystick.x2_axis);	   
   SPI_Send_Data(joystick.y2_axis);	    
   
   PORTD |= (1 << SS_PIN); 	     /* Chip Select OFF */ 
   
   PORTB |= (1 << CE_PIN);         /* Set CE HIGH to transmit */
   _delay_us(15);			           /* 15 µs CE pulse to trigger TX */
   PORTB &= ~(1 << CE_PIN);        /* Set CE LOW to return to Standby-I after packet is transmitted */               	 	
}  


/**
 * @brief  Read a telemetry payload from the RX FIFO (received via ACK).
 *         Assumes data is available (received_telem flag set).
 *
 * @param  data  Pointer to destination buffer.
 * @param  len   Number of bytes to read.
 */
 void get_Telem_Data(uint8_t* data, uint8_t len)
 {
   uint8_t i;
   PORTD &= ~(1 << SS_PIN);        	 /* Pull CSN low to begin SPI transaction */
   SPI_Send_Data(R_RX_PAYLOAD);    	 /* Read RX payload */
   for(i=0;i<len;i++)
   {
      SPI_Receive_Data(NOP,&data[i]);
   }						  
   PORTD |= (1 << SS_PIN);         	 /* Release CSN */
 }


