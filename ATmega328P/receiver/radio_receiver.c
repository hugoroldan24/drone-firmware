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
 * on the ATmega328P in receiver (PRX) mode. It handles IRQ notification (INT0) when new joystick
 * data arrives, reads joystick payloads from the RX FIFO via SPI, and supports Enhanced ShockBurst
 * features to attach telemetry data as an ACK payload (PRX -> PTX) using W_ACK_PAYLOAD.
 *
 * The receiver listens on pipe 0 for joystick frames and, when the transmitter requests an ACK,
 * the module can preload the PRX TX FIFO with telemetry so the next ACK includes that payload.
 * This requires activating features and enabling Dynamic Payload Length (DPL) and ACK payloads.
 *
 * Functions:
 *   - writeRegister
 *   - readRegister
 *   - sendCommand
 *   - writeAddress
 *   - RF_Receiver_Init
 *   - Radio_Listen
 *   - get_Received_Data
 *   - send_ACK_Payload
 *
 * Interrupt Service Routines:
 *   - ISR(INT0_vect)
 *
 ***********************************************************************************************/

#include "common.h"
#include "spi.h"
#include "delay.h"
#include <avr/interrupt.h>

volatile int8_t availableData = 0;


/**
 * @brief  INT0 (IRQ) interrupt service routine for nRF24L01+ receiver.
 *         Triggered on a falling edge when the RF module asserts IRQ indicating that an
 *         RX event occurred (e.g., payload received). The main loop should read the RX FIFO
 *         and clear the corresponding STATUS flags as needed.
 */
ISR(INT0_vect)
{
   availableData = 1;  /* Signal application that RF data/IRQ is pending */	
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
 * @brief  Initialize the nRF24L01+ as a receiver (PRX) on pipe 0 and enable ACK payload support.
 *         Sets up the IRQ pin on INT0, configures RF channel/bitrate/address width, enables pipe 0,
 *         activates special features (ACTIVATE + 0x73) and enables EN_ACK_PAY + EN_DPL, then enables
 *         dynamic payload on the desired pipes via DYNPD (required when using DPL in PRX mode).
 *
 *         Note: ACK payloads are queued in the PRX TX FIFO and are transmitted only when the next
 *         received packet actually triggers an ACK exchange; the MCU preloads the payload using the
 *         W_ACK_PAYLOAD command. This mechanism requires DPL to be enabled.
 */
void RF_Receiver_Init()
{
  uint8_t rx_pipe0_address[] = {0xE7,0xE7,0xE7,0xE7,0xE7};
  uint8_t tx_address[] = {0xE7,0xE7,0xE7,0xE7,0xE7};

  /* --- Configure INT0 (PD2) as the nRF24L01+ IRQ input (active low) --- */
  DDRD  &= ~(1 << DD_INT0);                  /* INT0 as input */
  PORTD |= (1 << INT0_PIN);		            /* Enable pull-up on PD2 */
  EICRA |= (1 << ISC01);                     /* Trigger INT0 on falling edge */ 
  EIFR  |= (1 << INTF0);			            /* Clear any pending INT0 flag */
  EIMSK |= (1 << INT0);  		               /* Unmask INT0 interrupt */
  
  /* --- CE pin: controlled by MCU to enter RX/TX states --- */
  DDRB  |= (1 << DD_CE);		                  /* Configure CE pin as output */
  PORTB &= ~(1 << CE_PIN);		               /* Ensure CE = 0 */
  
  _delay_us(10300);			    		          /* Wait ~10.3 ms for power-down stabilization */

  /* --- Basic RF setup --- */
  writeRegister(W_RF_CH,0x04);         	    /* Set RF channel to 2.404 GHz (avoid Wi-Fi) */	  
  writeRegister(W_RF_SETUP,0x0F);	    	  	 /* 2 Mbps data rate, 0 dBm, enable LNA gain */ 
  writeRegister(W_SETUP_AW,0x03);	   		 /* Address width = 5 bytes */ 
  //writeRegister(W_EN_AA,0x00);	             /* Disable auto-acknowledgment on all pipes */
  writeRegister(W_EN_AA,EN_AA_P0);          /* Enable data auto acknowledgment on pipe 0*/
  writeRegister(W_EN_RXADDR,0x01);	    		 /* Enable only pipe 0 */
  //writeRegister(W_RX_PW_P0,0x04);           /* Static payload length = 4 bytes on pipe 0 */ (no trabajamos con static)
  writeRegister(W_STATUS,0x7E);          	 /* Clear RX_DS flag and all the flags */  

  /* Pipe 0 address (must match transmitter). */
  writeAddress(W_RX_ADDR_P0,rx_pipe0_address,ADDRESS_WIDTH);      /* Set pipe 0 address */
  writeAddress(W_TX_ADDR,tx_address,ADDRESS_WIDTH);      /* Set transmitter address */

  writeRegister(ACTIVATE,ACTIVATION_KEY);    		               /* Activate features (enable W_ACK_PAYLOAD) */
  writeRegister(W_FEATURE,(1<<EN_ACK_PAY_BIT) | (1<<EN_DPL_BIT));	     		         /*  Enables Payload with ACK and Dynamic Length*/

  /* In PRX with DPL enabled, DYNPD must enable DPL per pipe (set at least pipe 0). [web:10] */
  writeRegister(W_DYNPD, (1 << DPL_P0_BIT)); 

  /* Auto Acknowledgent in pipe 0 is set by default */
  
  /* Flush FIFOs to start from a known state (important when changing modes/features). */
  sendCommand(FLUSH_TX);      
  sendCommand(FLUSH_RX);		    		                              

  /* --- Power up in PRX mode --- */
  writeRegister(W_CONFIG,0x3B);                       		      /* PWR_UP=1, PRIM_RX=1, CRC enabled, mask interrupts */
  _delay_us(1500);			    		                              /* Power-up to Standby-I timing */

  /* After configuration, module is in Standby-I until CE=1 */

 }
 
 
 /**
 * @brief  Pull CE high to begin listening on the configured RX pipe.
 *         After ~130 µs, the nRF24L01+ will start receiving on the set channel.
 */
 void Radio_Listen()
 {
  PORTB |= (1 << CE_PIN); /* CE high */	
  _delay_us(130);       /* RX Setting delay*/			    
 }	    
 
 
/**
 * @brief  Read one joystick frame from the RX FIFO (payload).
 *         Assumes a payload is available (availableData set, and/or STATUS indicates RX_DR).
 *         With dynamic payload enabled, the actual length should match what the transmitter sends.
 *
 * @param  joystick  Pointer to destination struct for decoded axes bytes.
 */
 void get_Received_Data(JoystickData *joystick)
 {
   PORTD &= ~(1 << SS_PIN);        	          /* CSN low: begin SPI transaction */
   SPI_Send_Data(R_RX_PAYLOAD);    	          /* Read RX payload command */

   /* Read joystick axes (4 bytes). */						  
   SPI_Receive_Data(NOP,&joystick->x1_axis);  
   SPI_Receive_Data(NOP,&joystick->y1_axis);  
   SPI_Receive_Data(NOP,&joystick->x2_axis);  
   SPI_Receive_Data(NOP,&joystick->y2_axis);  

   PORTD |= (1 << SS_PIN);         	         /* CSN high: end SPI transaction */
 }


 /**
 * @brief  Preload telemetry data into the PRX TX FIFO to be sent as ACK payload.
 *         The written payload remains pending in the PRX TX FIFO until a packet is received that
 *         triggers an ACK exchange.
 *
 * @param  payload  Pointer to telemetry bytes to attach to the ACK.
 * @param  len      Number of bytes to write (must not exceed the radio's max payload size).
 */
 void send_ACK_Payload(uint8_t *payload, uint32_t len)
 {
   PORTD &= ~(1 << SS_PIN); 
   SPI_Send_Data(W_ACK_PAYLOAD_P0);   /* Load ACK payload command in pipe 0 */
   for(uint8_t i = 0;i<len;i++){
         SPI_Send_Data(*payload++);
   }
   PORTD |= (1 << SS_PIN);
 }


