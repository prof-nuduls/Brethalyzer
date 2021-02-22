/*!
 * breathalyzer.c
 *
 *      Author: ece230
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "breathalyzer.h"
#include "lcd.h"
#include "delays.h"
#include "rgbLED.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
int i;

void ConvertValues(int voltage){


}
void promptUser(void){
    updateScreen("Warming Up...   ","                ");
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
    updateScreen("Begin Blowing   ","into Brethalyzer");
}
void readValuesAndStop(int *count){
    delaySeconds(5);
    MAP_Interrupt_enableInterrupt(INT_TA2_0);
    while (*count <= 50);
    MAP_Interrupt_disableInterrupt(INT_TA2_0);
}
void printValues(float AnalogValues){
    char analog[16];
    AnalogValues = AnalogValues/50;
    //conversion = (-3 * 10^5)*(ln(ppm))*(2*10^6);
    long double resistance = (abs((5-AnalogValues)/((AnalogValues/100000))));
    long double ratio = resistance/((2.2)*pow(10,6));
    long double ppmBAC = .00383*pow((1/ratio),(5000/1041))-.003;
    //long long bro = (exp((-1.51*(10^-6))*resistance));
    updateScreen("  Stop Blowing  ","                ");
    RGBLED_setColor(BLUE);
    delaySeconds(2);
    sprintf((char*)analog,"      %.2f %%     ",ppmBAC);
    updateScreen("Your BAC is:    ",analog);
}
