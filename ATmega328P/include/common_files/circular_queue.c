#include <stdint.h>

static const uint8_t size = CIRCULAR_QUEUE_SIZE;
static uint8_t num_queue_elem = 0;
static uint8_t circular_queue[size] /* Ha de ser múltiplo del tamaño de los paquetes de telemetria (24 bytes en este caso) */
static volatile uint8_t head = 0U; /* Indicates the position in the circular queue where next byte will be written */
static volatile uint8_t tail = 0U; /* Indicates the position from the next byte will be read */

static uint8_t get_num_elem(void);



// PIENSA QUE TU VAS REESCRIBIENDO TODO EN LA COLA, ESTO PUEDE LLEVAR A PROBLEMAS CON EL TAIL Y EL HEAD. ADEMAS PUEDE HABER PROBLEMAS 
// DE CONCURRENCIA Y TAL, VIGILAR.

void add_element_queue(uint8_t elem)
{
    circular_queue[head++] = elem;    /* Read the received byte */
    head = (head == size) ? 0 : head;
}

uint8_t read_element_queue(uint8_t *buf, uint8_t len)
{
   uint8_t num_queue_elem = get_num_elem();
   if(num_queue_elem == len){
      uint8_t new_tail = tail + len;
      memcpy(buf,&circular_queue[tail],len); /* Poblamos len bytes desde la direccion de memoria del elemento en la posicion tail de la circular queue*/
      tail = (new_tail >= size ) ? new_tail - size : new_tail; /* Actualizamos el tail*/
      return 1;
   }
   return 0;
}

static uint8_t get_num_elem()
{
    return (head-tail<0) ? (head-tail) + size : (head-tail);
}

