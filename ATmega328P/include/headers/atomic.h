#ifndef ATOMIC_H
#define ATOMIC_H

#include "common.h"

static inline uint8_t atomic_enter(void)
{   
   uint8_t sreg = SREG;    /* Save current state */
   SREG &= ~(1 << SREG_I); /* Disable interrupts */
   return sreg;
}

static inline void atomic_exit(uint8_t sreg)
{
   SREG = sreg; /* Restore previous state */
}

#endif
