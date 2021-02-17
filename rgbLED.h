/*! \file */
/*!
 * rgbLED.h
 *
 * Description: RGB driver using DriverLib for LED2 on MSP432P401R Launchpad
 *
 *  Created on:
 *      Author:
 */

#ifndef RGBLED_H_
#define RGBLED_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define RGB_PORT                                                    GPIO_PORT_P2
#define RGB_RED_PIN                                                 GPIO_PIN0
#define RGB_GREEN_PIN                                               GPIO_PIN1
#define RGB_BLUE_PIN                                                GPIO_PIN2
#define RGB_ALL_PINS                                                (0x0007)

typedef enum _Colors {
    BLACK,          /* (0x0000) */
    RED,            /* (0x0001) */
    GREEN,          /* (0x0002) */
    YELLOW,         /* (0x0003) */
    BLUE,           /* (0x0004) */
    MAGENTA,        /* (0x0005) */
    CYAN,           /* (0x0006) */
    WHITE           /* (0x0007) */
} Colors;

/*!
 * \brief This function configures LED2 pins as output pins
 *
 * This function configures P2.0, P2.1, and P2.2 as output pins
 *  for the RGB LED2, and initializes LED to 'off'
 *
 * Modified bits 0 to 2 of \b P2DIR register and \b P2SEL registers.
 *
 * \return None
 */
extern void RGBLED_init(void);


/*!
 * \brief This function toggles LED2 Red
 *
 * This function toggles the red LED of LED2
 *
 * Modified bit 0 of \b P2OUT register.
 *
 * \return None
 */
extern void RGBLED_toggleRed(void);


/*!
 * \brief This function toggles LED2 Green
 *
 * This function toggles the green LED of LED2
 *
 * Modified bit 1 of \b P2OUT register.
 *
 * \return None
 */
extern void RGBLED_toggleGreen(void);


/*!
 * \brief This function toggles LED2 Blue
 *
 * This function toggles the blue LED of LED2
 *
 * Modified bit 2 of \b P2OUT register.
 *
 * \return None
 */
extern void RGBLED_toggleBlue(void);


/*!
 * \brief This function sets color of LED2
 *
 * This function sets value for red, blue, and green LED outputs
 *  of LED2 based on \a newColor
 *
 *  \param newColor Color to set LED2
 *          Valid values are:
 *          - \b BLACK      (0x00)
 *          - \b RED        (0x01)
 *          - \b GREEN      (0x02)
 *          - \b YELLOW     (0x03)
 *          - \b BLUE       (0x04)
 *          - \b MAGENTA    (0x05)
 *          - \b CYAN       (0x06)
 *          - \b WHITE      (0x07)
 *
 * Modified bit 0 through 2 of \b P2OUT register.
 *
 * \return None
 */
extern void RGBLED_setColor(Colors color);



//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* RGBLED_H_ */
