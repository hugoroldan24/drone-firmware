#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "stm32f411xe.h"

void 	uart1_txrx_init_dma(void);
void 	uart1_send_dma(uint8_t *data, uint32_t len);
uint8_t uart1_read_dma(uint8_t* buf, uint16_t len);

#endif /* UART_H_ */

