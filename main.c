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
 *******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "delays.h"
#include "lcd.h"
#include "RFID.h"
#include "rgbLED.h"
#define TIMER_PERIOD    60000
#define MIN_ANGLE       1625
#define MAX_ANGLE       7250 + 938
#define TEN_DEGREE      1094
#define NOTEG3          7654
//Timer A PWM configuration
Timer_A_PWMConfig pwmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        TIMER_PERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        MIN_ANGLE
};
/* Port mapper configuration register */
const uint8_t port_mapping[] =
{
 //Port P2:
 /* DONE   modify pmap value for TA0CCR0 */
 PMAP_NONE, PMAP_NONE, PMAP_NONE, PMAP_NONE,PMAP_NONE,PMAP_TA0CCR0A,
 PMAP_NONE, PMAP_NONE
};
const Timer_A_UpModeConfig upConfig =
{
 TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock SOurce
 /* DONE change clock divider */
 TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 3MHz
 /* DONE change initial CCR0 value */
 0,                                      // 0 tick period (NOT USED)
 TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
 TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
 TIMER_A_DO_CLEAR                        // Clear value
};

const Timer_A_CompareModeConfig compareConfig_PWM =
{
 TIMER_A_CAPTURECOMPARE_REGISTER_0,          // Use CCR0
 TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
 TIMER_A_OUTPUTMODE_TOGGLE,                  // Toggle output
 NOTEG3                                      //Begin with frequency of G3 in CCR0
};
int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN3
    | GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    /* Setting the external clock frequency. This API is optional, but will
     * come in handy if the user ever wants to use the getMCLK/getACLK/etc
     * functions
     */
    CS_setExternalClockSourceFrequency(32000, 48000000);
    /* Starting HFXT in non-bypass mode without a timeout. Before we start
     * we have to change VCORE to 1 to support the 48MHz frequency */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_startHFXT(false);
    /* Initializing MCLK to HFXT (effectively 48MHz) */
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_4);
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16);
    // Configure and Initialize LCD
    configLCD(GPIO_PORT_P2, GPIO_PIN6, GPIO_PORT_P2, GPIO_PIN7, GPIO_PORT_P4);
    initDelayTimer(CS_getMCLK());
    initLCD();
    // Set S1 as input
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    // Initialize RGB LED
    RGBLED_init();
    //P2.4 as Output for Servo
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
              GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configuring Timer_A to have a period of approximately 500ms and
         * an initial duty cycle of 10% of that (3200 ticks)  */
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    MAP_PMAP_configurePorts((const uint8_t *) port_mapping, PMAP_P3MAP, 1,
                                    PMAP_DISABLE_RECONFIGURATION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,
                                      GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_Timer_A_initCompare(TIMER_A1_BASE, &compareConfig_PWM);
    /* Configuring Timer_A1 for UpDown Mode and starting */
    /* DONE change timer to TA0, use new struct, change mode */
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

    while (1)
    {

    }
}
