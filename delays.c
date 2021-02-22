/*!
 * delays.c
 *      Description: Helper file for delay functions using syTick timer. Must be
 *                   initialized with system clock frequency using initDelayTimer.
 *
 *      Author: ece230
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "delays.h"

#define USEC_DIVISOR    1000000
#define MSEC_DIVISOR    1000
#define SYSTICK_LIMIT   0x00FFFFFF

/* Holds frequency of system clock, must be set in initDelayTimer */
uint64_t sysClkFreq = 0;

void initDelayTimer(uint32_t clkFreq) {
    sysClkFreq = clkFreq;
}

int delayMicroSec(uint32_t micros) {
    uint64_t ticks = sysClkFreq * micros / USEC_DIVISOR;
    if (ticks < 2) {
        return UNDERFLOW;
    }
    if (ticks > SYSTICK_LIMIT) {
        return OVERFLOW;
    }

    // Set the period of the SysTick counter
    SysTick->LOAD = ticks - 1;
    // Write any value to reset timer counter
    SysTick->VAL = 1;
    SysTick_enableModule();
    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    SysTick_disableModule();
    return SUCCESS;
}

int delayMilliSec(uint32_t millis) {
    return delayMicroSec(1000 * millis);
}
void delaySeconds(uint32_t seconds){
    int i;
    for (i = 0; i < seconds; i++){
        delayMilliSec(1000);
    }
}
