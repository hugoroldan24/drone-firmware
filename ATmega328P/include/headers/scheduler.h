#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

typedef void (*task_func_t)(void);

typedef struct {
    task_func_t task;
    uint32_t period_ms;
    uint32_t elapsed_ms;
} scheduler_task_t;

void scheduler_init(void);
void scheduler_dispatch(void);
uint8_t scheduler_add_task(task_func_t task, uint32_t period_ms);

#endif
