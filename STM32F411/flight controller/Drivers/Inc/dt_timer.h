#ifndef DT_TIMER_H
#define DT_TIMER_H

#include <stdint.h>

void dt_timer_init(void);
uint32_t get_time_now_us(void);
float get_time_elapsed(uint32_t* lastcall);

#endif


