/*! \file */
/*!
 * rgbLED.c
 *
 * Description: RGB driver using DriverLib for LED2 on MSP432P401R Launchpad
 *
 *  Created on:
 *      Author:
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "rgbLED.h"


void RGBLED_init(void) {
    GPIO_setAsOutputPin(GPIO_PORT_P2,RGB_ALL_PINS);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,RGB_ALL_PINS);
}

void RGBLED_toggleRed(void) {
    MAP_GPIO_toggleOutputOnPin(RGB_PORT, RGB_RED_PIN);
}

void RGBLED_toggleGreen(void) {
    MAP_GPIO_toggleOutputOnPin(RGB_PORT, RGB_GREEN_PIN);
}

void RGBLED_toggleBlue(void) {
    MAP_GPIO_toggleOutputOnPin(RGB_PORT, RGB_BLUE_PIN);
}

void RGBLED_setColor(Colors color) {
    // TODO set the R, G, and B output values based on color value
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,RGB_ALL_PINS);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2,color);
}
