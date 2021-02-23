/*
 * speaker.h
 *
 *  Created on: Feb 22, 2021
 *      Author: millerd7
 */

#ifndef SPEAKER_H_
#define SPEAKER_H_
/*
 *\brief play a low frequency tone
 *\param none
 *\returns none
 */
extern void playTone(void);
/*
 *\brief stops any tone being played
 *\param none
 *\returns none
 */
extern void stopTone(void);
/*
 *\brief plays a high frequency tone to indicate wake up
 *\param none
 *\returns none
 */
extern void wakeUp(void);
/*
 *\brief play a tone to indicate to a user to start blowing
 *\param none
 *\returns none
 */
extern void middleTone(void);

#endif /* SPEAKER_H_ */
