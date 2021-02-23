/*
 * breathalyzer.h
 *
 *  Created on: Feb 19, 2021
 *      Author: millerd7
 */

#ifndef BREATHALYZER_H_
#define BREATHALYZER_H_
/*
 *\brief Converts Analog voltages to BAC percentage
 *\param analog voltage value you want to convert
 *\return BAC percentage
 */
extern long double ConvertValues(float AnalogValues);
/*
 *\brief tells you whether the BAC is above or below 0.08
 *\param BAC
 *\returns boolean 1 if below 0.08 and 0 if above 0.08
 */
extern bool car(long double BAC);
/*
 * \brief displays a start message
 * \param none
 * \returns none
 */
extern void start(void);
/*
 *\brief plays a tone on the speaker and displays a message on the screen to start blowing
 *\param none
 *\returns none
 */
extern void beginBlowing(void);
/*
 *\brief displays a warming up animation and tells user to press start
 *\param none
 *\returns none
 *
 */
extern void promptUser(void);
/*
 *\brief sets color of LED to red then displays a message on the screen to wait
 *for a green LED and switches the LED to green after 5 seconds
 *\param none
 *\returns none
 *
 */
extern void Wait (void);
/*
 *\brief turns brethalyzer module on and off by writing to the EN pin
 *\param none
 *\returns none
 */
extern void brethalyzerOff(void);
extern void brethalyzerOn(void);
/*
 *\brief waits 5 seconds before recording analog voltage samples
 *
 * waits five seconds enables TimerA2 interrupt gets 50 samples and then
 * turns off the TimerA2 interrupt
 * \param none
 *\returns none
 *
 */
extern void readValuesAndStop(int *count);
/*
 *\brief tells user to stop blowing turn LED blue and play a tone,
 *while displaying BAC
 *\param none
 *\returns none
 */
extern void printValues(float AnalogValues);
#endif /* BREATHALYZER_H_ */

