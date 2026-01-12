#include "common.h"
#include "radio_transmitter.h"
#include "adc.h"
#include "timer0.h"
#include "scheduler.h"

extern volatile uint8_t received_telem;

void send_data_task(void)
{
   static JoystickData joystick;	           /* Declares the union that contains the joystick data (defined in common.h file) */

   static uint8_t time_elapsed;
   time_elapsed++;  /* Con esto podemos contar de 1 ms en 1 ms*/

   uint8_t success;
   success = read_element_queue(joystick.axis,NUM_ELEMENTS);
   
   if(success){
      if(time_elapsed == SEND_ACK_PERIOD_MS) /* Comprobamos si ha pasado el tiempo, si es el caso pues enviamos un paquete pidiendo ACK */
        {
           time_elapsed = 0U;
           sendPaquet(joystick,ACK);
        }
        else{
           sendPaquet(joystick,NO_ACK);    	 /* Sends the joystick union containing the converted data from both axis */  
        }
   }
   else{
      return;
   }
}

void telemetry_task(void)
{
   static uint8_t telem_data[TELEM_FRAME_SIZE];
  
   if(received_telem) /*No hacemos while para no bloquear, solo comprobamos que haya payload*/
   {
      received_telem = 0U;
      get_Telem_Data(telem_data,TELEM_FRAME_SIZE);
   }
}
