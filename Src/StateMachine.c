/*
 * StateMachine.c
 *
 *  Created on: 31 okt. 2018
 *      Author: Max Pettersson
 */

#include "StateMachine.h"

buffertT buffert;
buffPointerT buffPointer;

void buffInit(void)
{
	buffPointer.pBuffBegin = buffert.eventBuff;
	buffPointer.pBuffEnd = buffert.eventBuff;
}

void stateMachine1(void)
{

}

void writeBuff(statesE state)
{
	__disable_irq();
	if(buffPointer.pBuffEnd > &buffert.eventBuff[BUFFSIZE-1])
	{
		buffPointer.pBuffEnd = &buffert.eventBuff[0];
	}
	*buffPointer.pBuffEnd = state;
	buffPointer.pBuffEnd++;
	__enable_irq();
}

statesE readBuff(void)
{
	if(buffPointer.pBuffBegin != buffPointer.pBuffEnd)
	{
		__disable_irq();
		if(buffPointer.pBuffBegin > &buffert.eventBuff[BUFFSIZE-1])
		{
			buffPointer.pBuffBegin = &buffert.eventBuff[0];
		}
		statesE temp = *buffPointer.pBuffBegin;
		buffPointer.pBuffBegin++;
		__enable_irq();
		return temp;
	}
	return STANDBY;

}
