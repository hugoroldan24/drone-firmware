 
/***********************************************************************************************
 * main.c
 *
 * Brief Description:
 * This file contains the application entry point for a bare-metal, cooperative scheduler-based
 * system. It first creates a circular queue, it registers periodic tasks (e.g., data reception and telemetry handling),  
 * initializes the required peripherals for the receiver subsystem, initializes the scheduler, and then
 * continuously dispatches scheduled tasks in the main loop.
 *
 * Functions:
 *   - int main(void);
 *
 ***********************************************************************************************/
 
#include "receiver.h"
#include "scheduler.h"
#include "rx_tasks.h"
#include "circular_queue.h"
#include "const.h"


CircularQueue receiver_cq;
CircularQueue *receiver_cq_ptr = &receiver_cq;
volatile uint8_t queue[CIRCULAR_QUEUE_SIZE_RX];


int main(void)
{  
   /* Initialice the circular queue that will be used for the telemetry system */
   create_circular_queue(receiver_cq_ptr,(uint8_t)CIRCULAR_QUEUE_SIZE_RX,queue);
   
   /* Register periodic tasks before starting the scheduler.
   Cast to void to explicitly ignore the return value (e.g., task ID / status code). */
   (void)scheduler_add_task(receive_data_task,RECEIVE_DATA_TASK_PERIOD_MS);
   (void)scheduler_add_task(telemetry_task,RX_TELEMETRY_TASK_PERIOD_MS);
 
   /* Initialize hardware/peripherals needed by the scheduled tasks
      and put the receiver into listening mode. */
   receiver_config();
  
   /* Initialize scheduler internal state/timers after hardware is ready. */
   scheduler_init();

   /* Main cooperative loop: run ready tasks; must never block for long periods. */
   while(1){
     scheduler_dispatch();
   }    
   return 0;	
}





