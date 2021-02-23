/*!
 * lcd.c
 *
 *      Description: Helper file for LCD library. For Hitachi HD44780 parallel LCD
 *               in 8-bit mode.
 *
 *      Author: ece230
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "speaker.h"
#include "delays.h"

void playTone(void){
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    MAP_Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                       7654);
}

void stopTone(void){
    MAP_Timer_A_stopTimer(TIMER_A1_BASE);
}
void middleTone(void){
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    MAP_Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                           4550/2);

}
void wakeUpTone(void){
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    MAP_Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0,
                                          4551);
}
void wakeUp(void){
    wakeUpTone();
    delayMilliSec(250);
    stopTone();
    delayMilliSec(250);
    wakeUpTone();
    delayMilliSec(250);
    stopTone();
    delayMilliSec(250);



}
