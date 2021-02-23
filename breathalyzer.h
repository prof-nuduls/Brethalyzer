/*
 * breathalyzer.h
 *
 *  Created on: Feb 19, 2021
 *      Author: millerd7
 */

#ifndef BREATHALYZER_H_
#define BREATHALYZER_H_

extern long double ConvertValues(float AnalogValues);
/*
 *
 */
extern bool car(long double BAC);
extern void start(void);
/*
 *
 *
 */
extern void beginBlowing(void);
/*
 *
 *
 */
extern void promptUser(void);
/*
 *
 *
 *
 */
extern void Wait (void);
/*
 *
 *
 *
 *
 */
extern void brethalyzerOff(void);
/*
 *
 *
 */
extern void brethalyzerOn(void);
/*
 *
 *
 */
extern void readValuesAndStop(int *count);
/*
 *
 */
extern void printValues(float AnalogValues);
/*
 *
 */
#endif /* BREATHALYZER_H_ */

