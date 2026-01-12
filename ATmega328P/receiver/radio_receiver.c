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
 * on the ATmega328P in receiver mode. It includes:
 *   - An ISR to detect when data arrives in the RX FIFO (via the IRQ pin).
 *   - Functions to write/read RF registers, send commands, set pipe addresses.
 *   - Initialization of the nRF24L01+ as a receiver on pipe 0 with a fixed 2-byte payload.
 *   - A function to enable the RF “listen” state (raise CE).
 *   - A function to read two bytes from the RX FIFO into provided buffers.
 *
 * Functions:
 *   - void writeRegister(uint8_t writeCommand, uint8_t conf);
 *       Writes a single-byte configuration value to the specified nRF24L01+ register.
 *
 *   - uint8_t readRegister(uint8_t reg);
 *       Reads and returns the value of the specified nRF24L01+ register.
 *
 *   - void sendCommand(uint8_t command);
 *       Sends a single-byte command to the nRF24L01+ (e.g., FLUSH_RX).
 *
 *   - void writeAddress(uint8_t pipe, uint8_t *addr, uint8_t size);
 *       Writes a multi-byte address to the specified RX or TX pipe register.
 *
 *   - void RF_Receiver_Init(void);
 *       Configures the nRF24L01+ as a receiver on pipe 0: sets channel, data rate, address
 *       width, payload size, disables auto-ACK, clears status flags, activates features,
 *       sets CONFIG for receiver mode, and flushes RX FIFO.
 *
 *   - void Radio_Listen(void);
 *       Pulls CE high to start listening on the configured pipe (valid after 130 µs).
 *
 *   - void get_Received_Data(uint8_t *byte1, uint8_t *byte2);
 *       Reads two bytes from the RX FIFO into the provided pointers by sending the
 *       R_RX_PAYLOAD command over SPI.
 *
 * Interrupt Service Routines:
 *   - ISR(INT0_vect):
 *       Triggered when nRF24L01+ asserts IRQ (falling edge on INT0). Sets availableData flag
 *       so the main loop knows data is ready to be read.
 *
 ***********************************************************************************************/
 
 
#include "common.h"
#include "spi.h"
#include "timer1.h"
#include <avr/interrupt.h>


volatile int8_t availableData = 0;


/**
 * @brief  INT0 (IRQ) interrupt service routine for nRF24L01+ receiver.
 *         Triggered on a falling edge when the RF module asserts IRQ indicating
 *         that new data is available in the RX FIFO. Sets availableData flag
 *         so that the main loop can call get_Received_Data().
 */
ISR(INT0_vect)
{
   availableData = 1;     /* Signal main code that new RF data is available */	
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
 * @param  command  The command you want to send to the module (e.g., FLUSH_RX, NOP).
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
 * @brief  Initialize the nRF24L01+ as a wireless receiver.
 *         Configures IRQ (INT0) pin, sets channel, data rate, address width,
 *         disables auto-ack, enables pipe 0 with a 2-byte payload, activates NO_ACK,
 *         sets CONFIG for RX mode (PWR_UP=1, PRIM_RX=1), and flushes RX FIFO.
 */
void RF_Receiver_Init()
{
  uint8_t rx_pipe0_address[] = {0xE7,0xE7,0xE7,0xE7,0xE7};
  /* Configure INT0 (PD2) for RF IRQ (active low) */
  DDRD  &= ~(1 << DD_INT0);                  /* INT0 as input */
  PORTD |= (1 << INT0_PIN);		               /* Enable pull-up on PD2 */
  EICRA |= (1 << ISC01);                    /* Trigger INT0 on falling edge */ 
  EIFR  |= (1 << INTF0);			                  /* Clear any pending INT0 flag */
  EIMSK |= (1 << INT0);  		                 /* Unmask INT0 interrupt */
  
  DDRB |= (1<<DD_CE);		                     /* Configure CE pin as output */
  PORTB &= ~(1 << CE_PIN);		                /* Ensure CE = 0 */
  
  delay_us(10300);			    		                /* Wait ~10.3 ms for power-down stabilization */
  writeRegister(W_RF_CH,0x04);         	    /* Set RF channel to 2.404 GHz (avoid Wi-Fi) */	  
  writeRegister(W_RF_SETUP,0x0F);	    	  	  /* 2 Mbps data rate, 0 dBm, enable LNA gain */ 
  writeRegister(W_SETUP_AW,0x03);	   		     /* Address width = 5 bytes */ 
  //writeRegister(W_EN_AA,0x00);	           		/* Disable auto-acknowledgment on all pipes */
  /writeRegister(W_EN_AA,EN_AA_P0);                 /* Enable data auto acknowledgment on pipe 0*/
  writeRegister(W_EN_RXADDR,0x01);	    		   /* Enable only pipe 0 */
  //writeRegister(W_RX_PW_P0,0x04);           /* Static payload length = 4 bytes on pipe 0 */ (no trabajamos con static)
  writeRegister(W_STATUS,(1<<6));          	/* Clear RX_DS flag */  
  writeAddress(W_RX_ADDR_P0,rx_pipe0_address,ADDRESS_WIDTH);  /* Set pipe 0 address */
  writeRegister(ACTIVATE,ACTIVATION_KEY);    		               /* Activate features (enable W_ACK_PAYLOAD) */
  writeRegister(W_FEATURE,EN_ACK_PAY | EN_DPL);	     		         /*  Enables Payload with ACK and Dynamic Length*/
  writeRegister(W_DYNPD, 0xFF); /* In RX mode the DYNPD has to be set */


            /* Estas tres
            /* ultimas lineas nos permiten usar el comando W_ACK_PAYLOAD */
            /* Tener en cuenta que tambien hay que tener habilitado el dynamic payload*/
            /* Siguendo el diagrama del receptor, aunque tengamos el Auto_ACK activado, si el paquete viene con flag NO_ACK activado*/
            /* El efecto será el mismo que si tuvieramos el Auto_ACK desactivado. De forma que cuando nos llegue un paquete con el flag*/
            /* NO_ACK desactivado, esta vez si enviaremos el ACK con la payload, podemos ir escribiendo en nuestro TX_FIFO la payload*/

  sendCommand(FLUSH_TX);      

  writeRegister(W_CONFIG,0x3B);                       		      /* PWR_UP=1, PRIM_RX=1, CRC enabled, mask interrupts */
  _delay_us(1500);			    		                                   /* Power-up delay ~1.5 ms */

  sendCommand(FLUSH_RX);		    		                              /* Flush RX FIFO */
 }
 
 
 /**
 * @brief  Pull CE high to begin listening on the configured RX pipe.
 *         After ~130 µs, the nRF24L01+ will start receiving on the set channel.
 */
 void Radio_Listen()
 {
  PORTB |= (1<<CE_PIN); /* CE high */	
  _delay_us(130);       /* RX Setting delay*/			    
 }	    
 
 
 /**
 * @brief  Read two bytes from the nRF24L01+ RX FIFO.
 *         Issues the R_RX_PAYLOAD command and reads two successive bytes into the
 *         provided pointers. Chip Select (CSN) is asserted low for the transaction.
 * @param  *joystick  Pointer to the struct where we will save the read bytes.
 */
 void get_Received_Data(JoystickData *joystick)
 {
   PORTD &= ~(1 << SS_PIN);        	         /* Pull CSN low to begin SPI transaction */
   SPI_Send_Data(R_RX_PAYLOAD);    	         /* Send command to read RX FIFO */							  
   SPI_Receive_Data(NOP,&joystick->x1_axis);  /* Read first byte (joystick X) */
   SPI_Receive_Data(NOP,&joystick->y1_axis);  /* Read second byte (joystick Y) */
   SPI_Receive_Data(NOP,&joystick->x2_axis);  /* Read first byte (joystick X) */
   SPI_Receive_Data(NOP,&joystick->y2_axis);  /* Read second byte (joystick Y) */
   PORTD |= (1 << SS_PIN);         	         /* Release CSN */
 }

 void send_ACK_Payload(uint8_t *payload, uint32_t len)
 {
   PORTD &= ~(1 << SS_PIN); 
   SPI_Send_Data(W_ACK_PAYLOAD);
   for(uint8_t i = 0;i<len;i++){
         SPI_Send_Data(*payload++)
   }
   PORTD |= (1 << SS_PIN);
 }

 void init_interrupt_pin()
 {
   DDRD  &= ~(1 << DD_INT1);                  /* INT1 as input */
   EICRA |= (1 << ISC11) | (1 << ISC10);      /* Trigger INT1 on rising edge */ 
   EIFR  |= (1 << INTF1);			            /* Clear any pending INT1 flag */
   EIMSK |= (1 << INT1);  		               /* Unmask INT1 interrupt */
 }