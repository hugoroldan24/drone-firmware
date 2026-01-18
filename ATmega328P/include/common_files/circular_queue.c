#include "atomic.h"
#include "common.h"
#include <stdint.h>

static volatile uint8_t circular_queue[CIRCULAR_QUEUE_SIZE];/* Ha de ser múltiplo del tamaño de los paquetes de telemetria (24 bytes en este caso) */
static volatile uint8_t head = 0U; /* Indicates the position in the circular queue where next byte will be written */
static volatile uint8_t tail = 0U; /* Indicates the position from the next byte will be read */

static inline uint8_t get_num_elem(void);
static inline uint8_t inc(uint8_t value);
static void memcpy_cq(uint8_t *dst, uint8_t len);

// Trabajamos con sobreescritura, si el siguiente head es tail, quiere decir que ya hemos dado la vuelta y la cola está llena, por tanto,
// nos deshacemos del elemento más antiguo (incrementando tail). Se añade lo de ISR porque si se hiciera en ejecución normal, deberias vigilar
// con accesos concurrentes. En una ISR este problema no se tiene.
void add_element_queue_ISR(uint8_t elem)
{  
   circular_queue[head] = elem;    
   head = inc(head);
   if(head == tail){
      tail = inc(tail);
   }
}  
   
uint8_t read_element_queue_atomic(uint8_t *buf, uint8_t len)
{
   uint8_t return_val = 0;
   
   uint8_t state = atomic_enter();
   
   uint8_t num_queue_elem = get_num_elem();
   if(num_queue_elem >= len){
      
      memcpy_cq(buf,len);
      return_val = 1;
   }
   atomic_exit(state);
   
   return return_val;
}

static void memcpy_cq(uint8_t *dst, uint8_t len)
{
   uint8_t i;
   for(i=0;i<len;i++){
      dst[i] = circular_queue[tail];
      tail = inc(tail);
   }
}

/* This function is donde considering that the CIRCULAR_QUEUE_SIZE <= 128, if that was not the case, special care should be taken with the
 data types! */
static inline uint8_t get_num_elem()
{  
   int8_t res = (int8_t) (head - tail);
   return (res<0) ? (uint8_t) (res + CIRCULAR_QUEUE_SIZE) : (uint8_t) res;
}

static inline uint8_t inc(uint8_t value)
{
   value ++;
   return ((value == CIRCULAR_QUEUE_SIZE) ? 0 : value);
}


