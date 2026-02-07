/***********************************************************************************************
 * scheduler.c
 *
 * Brief Description:
 * This module implements a simple cooperative, periodic task scheduler using Timer0 Compare Match A
 * interrupt (1 ms tick) to increment task timers. The main loop calls scheduler_dispatch() to execute
 * any tasks whose elapsed time meets or exceeds their configured period. Supports up to MAX_TASKS.
 *
 * Functions:
 *   - scheduler_init
 *   - scheduler_dispatch
 *   - scheduler_add_task
 *
 * Interrupt Service Routines:
 *   - ISR(TIMER0_COMPA_vect)
 *
 ***********************************************************************************************/


#include "scheduler.h"
#include "common.h"
#include <avr/interrupt.h>
#include <stdint.h>


static scheduler_task_t task_table[MAX_TASKS];
static uint32_t task_count;

static void Timer0_Start(void);
static void Timer0_Init(void);
static void scheduler_tick_isr(void);


/**
 * @brief  Timer0 Compare Match A interrupt service routine (1 ms tick).
 *         Delegates to scheduler_tick_isr() to increment all task elapsed timers.
 */
ISR(TIMER0_COMPA_vect)
{
   scheduler_tick_isr();
}


/**
 * @brief  Initialize the cooperative scheduler.
 *         Configures and starts Timer0 to generate 1 ms ticks for task timing.
 */
void scheduler_init()
{
   Timer0_Init();   /* Configure Timer0 CTC mode */
   Timer0_Start();  /* Start Timer0 with 1 ms period */
}


/**
 * @brief  Cooperative scheduler dispatch loop (called from main).
 *         Executes all tasks whose elapsed_ms >= period_ms, then resets their timers.
 *         Non-blocking; relies on caller to call frequently.
 */
void scheduler_dispatch(void)
{
    for (uint8_t i = 0; i < task_count; i++) {
        if (task_table[i].elapsed_ms >= task_table[i].period_ms) {
            task_table[i].elapsed_ms = 0;
            task_table[i].task();  
        }
    }
}


/**
 * @brief  Dynamically add a periodic task to the scheduler.
 *
 * @param  task       Function pointer to task entry point.
 * @param  period_ms  Task execution period in milliseconds.
 * @return 1 if task added successfully, 0 if table full (MAX_TASKS reached).
 */
uint8_t scheduler_add_task(task_func_t task, uint32_t period_ms)
{
    if(task_count >= MAX_TASKS) return 0;

    task_table[task_count].task = task;
    task_table[task_count].period_ms = period_ms;
    task_table[task_count].elapsed_ms = 0U;
    task_count++;
    
    return 1;
}


/**
 * @brief  1 ms scheduler tick handler (called from TIMER0_COMPA ISR).
 *         Increments elapsed_ms counter for all registered tasks.
 */
static void scheduler_tick_isr(void)
{
    for (uint8_t i = 0; i < task_count; i++) {
        task_table[i].elapsed_ms++;
    }
}


/**
 * @brief  Configure Timer0 in CTC mode for 1 ms tick generation.
 *         Sets OCR0A = SCHEDULER_PERIOD for desired tick rate.
 */
static void Timer0_Init()
{
  PRR &= ~(1 << PRTIM0);       /* Activate Timer0*/
  TCCR0A |= (1 << WGM01);      /* Set CTC mode */
  TCNT0 = 0U;
  OCR0A = SCHEDULER_PERIOD;
  TIMSK0 |= (1 << OCIE0A);      /* Unmask OCR0A compare match interrupt */
}


/**
 * @brief  Start Timer0 with prescaler for 1 ms tick generation.
 *         Prescaler 64: 16 MHz / 64 = 250 kHz (4 µs/tick), exact for 1 ms with OCR0A.
 */
static void Timer0_Start()
{
   TCCR0B |= (1 << CS01) | (1 << CS00);  /* Set prescaler to 16 MHz / 64 = 250 kHz to get an exact number of ticks for counting up to 1 ms*/   
}
