/*!
 * breathalyzer.c
 *      description: Header file to do the bulk of the calculations and LCD display
 *      for the Brethalyzer module.
 *      Author: millerd7
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "breathalyzer.h"
#include "lcd.h"
#include "delays.h"
#include "rgbLED.h"
#include "speaker.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

int i;
long double ppmBAC;

long double ConvertValues(float AnalogValues){
    AnalogValues = AnalogValues/50;
      long double resistance = (abs((3.3-AnalogValues)/((AnalogValues/200000))));
      long double ratio = resistance/((2)*pow(10,6));
      long double ppmBAC = fabs(.00383*pow((1/ratio),(5000/1041)));
      return ppmBAC;

}
bool car (long double BAC){
    if (BAC < 0.08){
        return 1;
    }
    else {
        return 0;
    }
}

void promptUser(void){
    updateScreen("Warming Up      ", "                ");
    delayMilliSec(500);
    updateScreen("Warming Up.     ", "                ");
    delayMilliSec(500);
    updateScreen("Warming Up..    ", "                ");
    delayMilliSec(500);
    updateScreen("Warming Up...   ", "                ");
    delayMilliSec(500);
    updateScreen("Device Ready    ","                ");
    delayMilliSec(1000);
    updateScreen("  Press Button  ","    to start    ");
}

void Wait(void){
    RGBLED_setColor(RED);
    updateScreen("    Wait for    ","  Green LED...  ");
    delaySeconds(6);
    RGBLED_setColor(GREEN);

}
void brethalyzerOff(void){
    GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6);
}
void brethalyzerOn(void){
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
}
void start(void){
    updateScreen("Brethalyzer     ","Module Start    ");
}
void beginBlowing(void){
    middleTone();
    updateScreen("Begin Blowing   ","into Brethalyzer");
    delaySeconds(1);
    stopTone();
}
void readValuesAndStop(int *count){
    delaySeconds(5);
    MAP_Interrupt_enableInterrupt(INT_TA2_0);
    while (*count <= 50);
    MAP_Interrupt_disableInterrupt(INT_TA2_0);

}
void printValues(float AnalogValues){
    char analog[16];
    long double v = ConvertValues(AnalogValues);
    int fix = (int) (v * 100);
    sprintf((char*)analog,"       .  %%     ");
    analog[6] = '0' + fix/100;
    analog[8] = '0' + (fix/10 % 10);
    analog[9] = '0' + (fix/1 % 10);
    delaySeconds(1);
    playTone();
    updateScreen("  Stop Blowing  ","                ");
    RGBLED_setColor(BLUE);
    delaySeconds(2);
    stopTone();
    updateScreen("Your BAC is:    ",analog);
}
