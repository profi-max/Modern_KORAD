/*
 * oscill.h
 *
 *  Created on: 27 янв. 2022 г.
 *      Author: profi-max
 */

#ifndef INC_OSCILL_H_
#define INC_OSCILL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

// global variables definitions
extern uint8_t gOscMode; //  1bit - V ray; 2bit - A ray; 3bit - A ray in front

//  export functions
void oscInit(void);
void oscInitColors(void);
void oscSetRange(void);
void oscLoop(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* INC_OSCILL_H_ */
