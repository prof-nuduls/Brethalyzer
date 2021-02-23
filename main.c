/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 Empty Project
 *
 * Description: An empty project that uses DriverLib
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 * Author: 
 *
 *
 *
 * LCD:
 * RS -> P2.6
 * EN -> P2.7
 * DB -> P4
 *
 * S1 -> P1.1
 * RGB -> P2.1, P2.2, P2.3
 * Servo -> P2.4
 * Brethalyzer:
 * EN -> P3.6
 * AO -> P6.0
 *******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
//include header files
#include "delays.h"
#include "lcd.h"
#include "RFID.h"
#include "rgbLED.h"
#include "breathalyzer.h"
#include "initClock.h"
#include "speaker.h"
typedef enum _SwitchState{Pressed, NotPressed} SwitchState;
/* Statics */
static volatile uint16_t curADCResult;
static volatile float normalizedADCRes;
float AnalogValues = 0;
#define SwitchPort GPIO_PORT_P1
#define SwitchS1   GPIO_PIN1
#define TIMER_PERIOD    60000
#define MIN_ANGLE       1625
#define MAX_ANGLE       7250 + 938
#define TEN_DEGREE      1094
#define NOTEG3          7654
int ii;
int count = 1;

SwitchState CheckS1(void);  //check Switch S1 and return Pressed or NotPressed
//Timer A PWM configuration
Timer_A_PWMConfig pwmConfig = {
TIMER_A_CLOCKSOURCE_SMCLK,
                                TIMER_A_CLOCKSOURCE_DIVIDER_1,
                                TIMER_PERIOD,
                                TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                TIMER_A_OUTPUTMODE_RESET_SET,
                                MIN_ANGLE };
/* Port mapper configuration register */
const uint8_t port_mapping[] = {
//Port P2:
        /* DONE   modify pmap value for TA0CCR0 */
        PMAP_NONE,PMAP_NONE,
        PMAP_TA0CCR0A, PMAP_NONE, PMAP_NONE, PMAP_NONE,
        PMAP_NONE,PMAP_NONE };
const Timer_A_UpModeConfig upConfigSpeaker = {
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock SOurce
        /* DONE change clock divider */
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 3MHz
        /* DONE change initial CCR0 value */
        0,                                      // 0 tick period (NOT USED)
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
        };
const Timer_A_UpModeConfig upConfigBrethalyzerRead =
{
 TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
 TIMER_A_CLOCKSOURCE_DIVIDER_16,          // SMCLK/16 = 1.875 kHz
 3750         ,                          // 10986 tick period
 TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
 TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
 TIMER_A_DO_CLEAR                        // Clear value
};
const Timer_A_UpModeConfig upConfigSpeaker1 =
{
 TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
 TIMER_A_CLOCKSOURCE_DIVIDER_64,         // SMCLK/64 = 46875 Hz
 46875/2,                                // 46875/2 tick period
 TIMER_A_TAIE_INTERRUPT_ENABLE,          // Enable Timer interrupt
 TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE ,   // Disable CCR0 interrupt
 TIMER_A_DO_CLEAR                        // Clear value
};

const Timer_A_CompareModeConfig compareConfig_PWM = {
TIMER_A_CAPTURECOMPARE_REGISTER_0,          // Use CCR0
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE,                  // Toggle output
        NOTEG3                             //Begin with frequency of G3 in CCR0
        };
int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();
    SwitchState S1Status;
    //Intialize MCLK, SMCLK, and ADC14
    curADCResult = 0;
    initADC14();
    initClocks();
    long long boi = CS_getSMCLK();
    // Configure and Initialize LCD
    configLCD(GPIO_PORT_P2, GPIO_PIN6, GPIO_PORT_P2, GPIO_PIN7, GPIO_PORT_P4);
    initDelayTimer(CS_getMCLK());
    initLCD4bit();
    //turn off brethalyzer
    brethalyzerOff();
    // Set S1 as input
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P6,GPIO_PIN0);
    // Initialize RGB LED
    RGBLED_init();
    //P2.4 as Output for Servo
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN2,
    GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configuring Timer_A to have a period of approximately 500ms and
     * an initial duty cycle of 10% of that (3200 ticks)  */
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    MAP_PMAP_configurePorts((const uint8_t*) port_mapping, PMAP_P3MAP, 1,
    PMAP_DISABLE_RECONFIGURATION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
            GPIO_PORT_P3,
            GPIO_PIN3,
            GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configuring Timer_A1 for UpDown Mode and starting */
    /* DONE change timer to TA0, use new struct, change mode */
    initADC14Module();
    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &upConfigBrethalyzerRead);
     /* Enabling interrupts */
   // MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
      MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
      /* Configuring Timer_A1 for UpDown Mode and starting */
      MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfigSpeaker);
      MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
      //playTone();
      //delaySeconds(5);
      //stopTone();


    //MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();
    S1Status = CheckS1();
    start();
    while (S1Status == NotPressed)
    {
        S1Status = CheckS1();
        if (S1Status == Pressed)
        {

            promptUser();
        }
    }
    S1Status = NotPressed;
    while (S1Status == NotPressed)
    {
        S1Status = CheckS1();
        if (S1Status == Pressed)
        {
            S1Status = NotPressed; //set S1Status as NotPressed
            brethalyzerOn();
            Wait();
            beginBlowing();
            readValuesAndStop(&count);
            brethalyzerOff();
            printValues(AnalogValues);
        }
    }
    //end if(S1Status==Pressed)
}

void debounce(void)
{
    for (ii = 0; ii < 100; ii++);
//delay time for debouncing switches
} //end debounce()

SwitchState CheckS1(void)
{
    char switchValue;
    switchValue = GPIO_getInputPinValue(SwitchPort, SwitchS1);
    if (switchValue == 0)
    {      //check S1.
        debounce(); //If pressed, debounce, check for button release, debounce, return Pressed
        while (switchValue == 0)
        {
            switchValue = GPIO_getInputPinValue(SwitchPort, SwitchS1);
            if (switchValue == 1)
            {
                debounce();
                return Pressed;
            }
        }
    }
    else if (switchValue == 1)
    {
        return NotPressed;
    }

}
void TA2_0_IRQHandler(void)
{
   count++;
   curADCResult = MAP_ADC14_getResult(ADC_MEM0);
   normalizedADCRes = ((curADCResult * 3.3 )/ 16383)-1.9;
   AnalogValues += normalizedADCRes;
   MAP_ADC14_toggleConversionTrigger();
   MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
                                             TIMER_A_CAPTURECOMPARE_REGISTER_0); //clear flag
}
