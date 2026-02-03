

#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include <stdint.h>

typedef struct {
   volatile uint8_t head;   /* Next write position */
   volatile uint8_t tail;   /* Next read position */
   volatile uint8_t *queue; /* Pointer to an array */
   uint8_t size;            /* Must be multiple of frame size */
} CircularQueue;

void    create_circular_queue(CircularQueue *cq, uint8_t size, volatile uint8_t *queue);
void    add_element_queue_ISR(uint8_t elem, CircularQueue *cq);
uint8_t read_element_queue_atomic(uint8_t *buf, uint8_t len, CircularQueue *cq);

#endif
