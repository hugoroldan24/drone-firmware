#include "spi.h"
#include "radio_receiver.h"
#include "common.h"
#include "telemetry.h"
#include "usart.h"

void receive_data_task(void)
{
  static JoystickData joystick;    		   /* Declares the union joystick (defined in common.h file) */
  while(!availableData);					  /* Wait until nRF24L01+ external interrupt sets availableData */    
  availableData = 0;  				 	  /* Reset flag */
  writeRegister(W_STATUS,(1<<RX_DS));      /* Clear RX_DS flag on nRF24L01+ */
     
  get_Received_Data(&joystick); 			  /* Read two bytes from RX_FIFO and saves it in the joystick variable */ 
     
  /* Send joystick data to the flight controller */
  USART_Send(joystick.axis, NUM_ELEMENTS);
}

void telemetry_task(void)
{
  static uint8_t *telem_ptr;
  do_telemetry(telem_ptr);
}
