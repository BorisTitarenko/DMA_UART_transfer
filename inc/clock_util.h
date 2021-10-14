#ifndef CLOCK_UTIL_H
#define CLOCK_UTIL_H

#include "stm32f4xx.h"
#include <system_stm32f4xx.h>


#define M   4
#define N   84
#define P   0

//Configure PLL using defines and set PLL as source clock
void SetClock(void);

// Set HSE as clock source, Sysclock will be 8 mHz
void SetClockHSE(void); 
#endif