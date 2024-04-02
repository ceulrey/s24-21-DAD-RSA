/*
 * DAD_SW_Timer.h
 *
 *  Created on: Apr 3, 2023
 *      Author: Max
 */

#ifndef DAD_SW_TIMER_H_
#define DAD_SW_TIMER_H_

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <HAL/DAD_Timer.h>

#ifdef SET_TIMER_3_AS_SW_TIMER

#define DAD_SW_TIMER_HANDLE         TIMER_A3_BASE
#define DAD_SW_TIMER_RESOLUTION_MS  1000            // How often (in ms) the timer updates

// Initializes the hardware timer.
void DAD_SW_Timer_initHardware();

// Get ms since starting (max of 32 bits)
bool DAD_SW_Timer_getMS(uint64_t* timeToReturn);

#endif // SET_TIMER_3_AS_SW_TIMER
#endif /* DAD_SW_TIMER_H_ */
