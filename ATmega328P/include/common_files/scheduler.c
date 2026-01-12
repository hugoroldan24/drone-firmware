
#include <stdint.h>
#include "scheduler.h"

static scheduler_task_t task_table[MAX_TASKS];
static uint32_t task_count;

static void Timer0_Start(void);
static void Timer0_Init(void);
static void scheduler_tick_isr(void);

ISR(TIMER0_COMPA_vect)
{
   scheduler_tick_isr();
}

void scheduler_init()
{
    Timer0_Init();
    Timer0_Start();
}

void scheduler_dispatch(void)
{
    for (uint8_t i = 0; i < task_count; i++) {
        if (task_table[i].elapsed_ms >= task_table[i].period_ms) {
            task_table[i].elapsed_ms = 0;
            task_table[i].task();  
        }
    }
}

uint8_t scheduler_add_task(task_func_t task, uint32_t period_ms)
{
    if(task_count >= MAX_TASKS) return 0;

    task_table[task_count].task = task;
    task_table[task_count].period_ms = period_ms;
    task_table[task_count].elapsed_ms = 0U;
    task_count++;
    return 1;
}

static void scheduler_tick_isr(void)
{
    for (uint8_t i = 0; i < task_count; i++) {
        task_table[i].elapsed_ms++;
    }
}

static void Timer0_Init()
{
  PRR &= ~(1 << PRTIM0);       /* Activate Timer0*/
  TCCR0A |= (1 << WGM01);      /* Set CTC mode */
  TCNT0 = 0U;
  OCR0A = SCHEDULER_PERIOD;
  TIMSK0 |= (1 << OCIE0A);      /* Unmask OCR0A compare match interrupt */
}

static void Timer0_Start()
{
   TCCR0B |= (1 << CS02) | (1 << CS00);  /* Set prescaler to 16 MHz / 64 = 250 kHz to get an exact number of ticks for counting up to 1 ms*/   
}