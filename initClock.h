/*
 * initClock.h
 *
 *  Created on: Feb 19, 2021
 *      Author: millerd7
 */

#ifndef INITCLOCK_H_
#define INITCLOCK_H_
/*
 * \brief set clock values and enable FPU for the ADC14
 * \param none
 * \returns none
 */
extern void initADC14(void);
/*
 *\brief initialize SMCLK and MCLK
 *\param none
 * \returns none
 *
 */
extern void initClocks(void);
/*
 *\brief initialize memory modules and analog pins for ADC14
 *\param none
 * \returns none
 */
extern void initADC14Module(void);

#endif /* INITCLOCK_H_ */
