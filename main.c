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
static volatile bool debounced;
float AnalogValues = 0;
#define SwitchPort GPIO_PORT_P1
#define SwitchS1   GPIO_PIN1
#define TIMER_PERIOD    60000
#define MIN_ANGLE       1625
#define UNLOCKED        3813
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
 TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/16 = 187.5 kHz
 3750         ,                          // 10986 tick period
 TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
 TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
 TIMER_A_DO_CLEAR                        // Clear value
};

Timer_A_UpModeConfig speakHoeBuzzConfig =
{
 TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock SOurce
 TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 48MHz
 23050,                                  // 23050 tick period
 TIMER_A_TAIE_INTERRUPT_ENABLE,          // Enable Timer interrupt
 TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR1 interrupt
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

    //this will be where we enter into sleep mode
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN5);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN5);
    MAP_Interrupt_enableInterrupt(INT_PORT1);
    MAP_Interrupt_enableMaster();
    bool sanitity = PCM_setPowerState(PCM_LPM45);

    // Set S1 as input
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    // Set ethanol pins
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P6,GPIO_PIN0);
    // Initialize RGB LED
    RGBLED_init();
    //P2.4 as Output for Servo
    /* Configuring Timer_A to have a period of approximately 500ms and
     * an initial duty cycle of 10% of that (3200 ticks)  */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
                GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    //MAP_PMAP_configurePorts((const uint8_t*) port_mapping, PMAP_P3MAP, 1, PMAP_DISABLE_RECONFIGURATION);
    //MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
    /* DONE change timer to TA0, use new struct, change mode */
    initADC14Module();

    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &upConfigBrethalyzerRead);

     /* Enabling interrupts */
    //timer 32 note durations setup/ control code
    MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_16BIT,
                           TIMER32_FREE_RUN_MODE);
    MAP_Timer32_enableInterrupt(TIMER32_0_BASE);
    //MAP_Timer32_startTimer(TIMER32_0_BASE, true);
    /* Enabling interrupts and starting the timer */
    MAP_Interrupt_enableInterrupt(INT_T32_INT1);

    // MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
    //pitch timer
    MAP_Timer_A_configureUpMode(TIMER_A3_BASE, &upConfigSpeaker);
    //pitch timer
    MAP_Timer_A_initCompare(TIMER_A3_BASE, &compareConfig_PWM);
    //start counter
    MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);

    //tom janko mode
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &speakHoeBuzzConfig);
    MAP_Interrupt_enableInterrupt(INT_TA1_N);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN2);

    //MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();
    S1Status = CheckS1();
    //start();
    /*while (S1Status == NotPressed)
    {
        S1Status = CheckS1();
        if (S1Status == Pressed)
        {

            promptUser();
        }
    }*/
    wakeUp();
    promptUser();
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
            delaySeconds(3);
            updateScreen("Processing      ", "                ");
            delayMilliSec(500);
            updateScreen("Processing.     ", "                ");
            delayMilliSec(500);
            updateScreen("Processing..    ", "                ");
            delayMilliSec(500);
            updateScreen("Processing...   ", "                ");
            delayMilliSec(500);
            int BAC = (ConvertValues(AnalogValues) * 100);
           if (8 > BAC) {
                    updateScreen("Your Car is     ","    UNLOCKED    ");
                    pwmConfig.dutyCycle = 3813;
                    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
                }
                else{
                    updateScreen("Your Car is     ","     LOCKED     ");
                }
        }
    }

    //end if(S1Status==Pressed)
}

void debounce(void)
{
    debounced = false;
    MAP_Timer32_setCount(TIMER32_0_BASE, 100);
    MAP_Timer32_startTimer(TIMER32_0_BASE, true);
    while(!debounced);

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

void T32_INT1_IRQHandler(void) //This interrupt will set the pitch period and the duration of the next note.
{
    MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);
    debounced = true;
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

void TA1_N_IRQHandler(void) //This will end the debounce delay and halt the debounce timer
{
    MAP_Timer_A_clearInterruptFlag(TIMER_A1_BASE);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P3,GPIO_PIN2);
}

/* GPIO ISR */
void PORT1_IRQHandler(void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    /* Toggling the output on the LED */
    if(status & GPIO_PIN5)
    {
        //keyless = false;
        //PCM_setPowerState(PCM_AM_LDO_VCORE0);
        //Interrupt_disableSleepOnIsrExit();
    }

}
