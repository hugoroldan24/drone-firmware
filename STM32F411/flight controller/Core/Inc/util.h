#ifndef UTIL_H_
#define UTIL_H_

void dt_timer_init(void);
uint32_t get_time_now_us(void);
float get_time_elapsed(uint32_t* lastcall);
void constrain(float* value, float low, float high);


#endif /* UTIL_H_ */


