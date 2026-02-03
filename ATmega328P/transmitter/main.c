/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
 /***********************************************************************************************
 * main.c
 *
 * Brief Description:
 * This file contains the application entry point for a bare-metal, cooperative scheduler-based
 * system. It first creates a circular queue, it registers periodic tasks, initializes
 * the required peripherals for the receiver subsystem, initializes the scheduler, and then
 * continuously dispatches scheduled tasks in the main loop.
 *
 * Functions:
 *   - int main(void);
 *
 ***********************************************************************************************/
 
 
#include "const.h"
#include "transmitter.h"
#include "circular_queue.h"
#include "scheduler.h"
#include "tx_tasks.h"


CircularQueue transmitter_cq;
CircularQueue *transmitter_cq_ptr = &transmitter_cq;
volatile uint8_t queue[CIRCULAR_QUEUE_SIZE_TX];


int main(void)
{
   /* Initialice the circular queue that will be used for saving the ADC values */
   create_circular_queue(transmitter_cq_ptr,(uint8_t)CIRCULAR_QUEUE_SIZE_TX,queue);
   
   /* Register periodic tasks before starting the scheduler.
   Cast to void to explicitly ignore the return value (e.g., task ID / status code). */   
   (void)scheduler_add_task(send_data_task, SEND_DATA_TASK_PERIOD_MS);
   (void)scheduler_add_task(telemetry_task, TX_TELEMETRY_TASK_PERIOD_MS);

   /* Initialize hardware/peripherals needed by the scheduled tasks */
   transmitter_config();	   	       

   /* Initialize scheduler internal state/timers after hardware is ready. */
   scheduler_init();

  /* Main cooperative loop: run ready tasks; must never block for long periods. */
   while(1){		
      scheduler_dispatch();
   }  
   return 0;
}


