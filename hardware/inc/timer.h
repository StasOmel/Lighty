#ifndef __TIMER_H
#define __TIMER_H

#include "system.h"


void timerInit(void);
void timerDelayDecrement(void);
void Delay(uint32_t nTime);
uint32_t timer10msTest(void);

void timerDelay_ms(unsigned int delay);
void timerDelay_us(unsigned int delay);

#endif /* __TIMER_H */
