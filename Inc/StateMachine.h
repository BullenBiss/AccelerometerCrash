/*
 * StateMachine.h
 *
 *  Created on: 31 okt. 2018
 *      Author: Max Pettersson
 */

#include "stm32f4xx_hal.h"

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_
#define BUFFSIZE 20

typedef enum {IDLE, ACTIVITY, STANDBY, ACTIVATE, TIMER, RESETT, REST} statesE;

typedef struct buffertT{
	statesE eventBuff[BUFFSIZE];
}buffertT;

typedef struct buffPointerT{
	statesE *pBuffBegin;
	statesE *pBuffEnd;
}buffPointerT;



void buffInit(void);
void stateMachine1(void);
void writeBuff(statesE state);
statesE readBuff(void);

#endif /* STATEMACHINE_H_ */
