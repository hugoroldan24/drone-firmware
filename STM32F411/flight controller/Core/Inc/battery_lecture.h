#ifndef BATTERY_LECTURE_H_
#define BATTERY_LECTURE_H_

#include <stdint.h>

void init_adc(void);
void start_battery_lecture(void);
uint16_t read_battery(void);

#endif /* BATTERY_LECTURE_H_ */

