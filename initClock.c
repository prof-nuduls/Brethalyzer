/*!
 * initClock.c
 *      Description: initialize clocks to save space in main.c
 *
 *      Author: millerd7 and TI
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include "initClock.h"

#include <stdint.h>
#include <stdbool.h>
void initADC14(void){

    /* Setting Flash wait state */
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 1);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 1);

    /* Setting DCO to 48MHz  */
    MAP_PCM_setPowerState(PCM_AM_LDO_VCORE1);
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);

    /* Enabling the FPU for floating point operation */
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking();

    //![Single Sample Mode Configure]
    /* Initializing ADC (MCLK/1/4) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4,
                         0);

}
void initClocks (void){
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(
               GPIO_PORT_PJ, GPIO_PIN3 | GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
       //MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
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
}

void initADC14Module(void){
    /* Configuring ADC Memory */
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
    ADC_INPUT_A15, false);

    /* Configuring Sample Timer */
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    /* Enabling/Toggling Conversion */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    MAP_ADC14_enableInterrupt(ADC_INT0);
}
