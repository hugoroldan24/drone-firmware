

#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include <stdint.h>

void add_element_queue(uint8_t elem);
uint8_t read_element_queue(uint8_t *buf, uint8_t len);

#endif