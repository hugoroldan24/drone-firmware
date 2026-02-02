

#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include <stdint.h>

typedef struct {
   volatile uint8_t head;  /* Next write position */
   volatile uint8_t tail;  /* Next read position */
   voaltile uint8_t *queue;
   uint8_t size;           /* Must be multiple of frame size */
} CircularQueue;

void    add_element_queue_ISR(uint8_t elem, CircularQueue *cq);
uint8_t read_element_queue_atomic(uint8_t *buf, uint8_t len, CircularQueue *cq);

#endif
