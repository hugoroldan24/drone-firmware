/***********************************************************************************************
 * circular_queue.c
 *
 * Brief Description:
 * This module implements a lock-free circular queue designed for ISR writes (ADC/USART data) and atomic multi-byte reads (complete joystick/telemetry frames) from main context. ISR writer uses overwrite policy: if full, discards oldest data. Atomic reads only consume complete frames when enough bytes are available.
 *
 * Functions:
 *   - add_element_queue_ISR
 *   - read_element_queue_atomic
 *
 ***********************************************************************************************/


#include "atomic.h"
#include "common.h"
#include "circular_queue.h"
#include <stdint.h>


static inline uint8_t get_num_elem(CircularQueue *cq);
static inline uint8_t inc(uint8_t value, CircularQueue *cq);
static void memcpy_cq(uint8_t *dst, uint8_t len, CircularQueue *cq);


/**
 * @brief  ISR-safe single-byte push to circular queue with overwrite policy.
 *         Always succeeds; if queue is full (head == tail after write), discards
 *         oldest byte by advancing tail.
 */
void add_element_queue_ISR(uint8_t elem, CircularQueue *cq)
{  
   cq->queue[cq->head] = elem;           /* Write new element */   
   cq->head = inc(cq->head,cq->size);    /* Advance write position */
   if(cq->head == cq->tail){
      cq->tail = inc(cq->tail,cq->size); /* Overwrite: discard oldest element */
   }
}  
 

/**
 * @brief  Atomic multi-byte read of complete frame from circular queue.
 *         Only consumes data if num_elements >= requested length (no partial frames).
 *
 * @param  buf  Destination buffer for frame data.
 * @param  len  Number of bytes required (e.g., NUM_ELEMENTS, TELEM_FRAME_SIZE).
 * @param  cq   Pointer to circular queue
 * @return 1 if full frame read successfully, 0 if insufficient data.
 */
uint8_t read_element_queue_atomic(uint8_t *buf, uint8_t len, CircularQueue *cq)
{
   uint8_t return_val = 0;
   
   uint8_t state = atomic_enter();
   
   uint8_t num_queue_elem = get_num_elem(cq);
   if(num_queue_elem >= len){
      
      memcpy_cq(buf,len,cq);
      return_val = 1;
   }
   atomic_exit(state);
   
   return return_val;
}


/**
 * @brief  Helper: copy 'len' bytes from circular queue to destination.
 *         Advances tail after each byte (called only when enough data guaranteed).
 */
static void memcpy_cq(uint8_t *dst, uint8_t len, CircularQueue *cq)
{
   uint8_t i;
   for(i=0;i<len;i++){
      dst[i] = cq->queue[cq->tail];
      cq->tail = inc(cq->tail,cq->size);
   }
}


/**
 * @brief  Calculate current number of elements in queue.
 *         Works for CIRCULAR_QUEUE_SIZE <= 128; uses uint8_t wraparound math.
 *         Assumes called within atomic context.
 */
static inline uint8_t get_num_elem(CircularQueue *cq)
{  
   int8_t res = (int8_t) (cq->head - cq->tail);
   return (res<0) ? (uint8_t) (res + cq->size) : (uint8_t) res;
}


/**
 * @brief  Increment index with wraparound to param size.
 *         Safe for both head (ISR writes) and tail (atomic reads).
 */
static inline uint8_t inc(uint8_t value, uint8_t size)
{
   value ++;
   return ((value == size) ? 0 : value);
}


