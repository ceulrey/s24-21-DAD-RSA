/*
 * DAD_Timer.h
 *
 *  Created on: Dec 31, 2022
 *      Author: Maximilien Engel


    Description :   Asynchronous timer
                    Timer is initialized with timer length
                    Timer is then started.
                    If timer stops before timer expires, nothing happens. Program continues as if timer was never set.
                    Else, set timer expired flag high

                    Only 4 timers can be initialized at a time

                    ms mode:
                    Timer runs at 1024Hz, which is approx 1ms period.
                    Max period of about 65535ms. Min period of about 2ms.

                    us mode:
                    Timer runs at SMCLK, which defaults to 3MHz.
                    Worst case max period is 1365us (at SMCLK = 48Mhz). Min period of 1us.
 */

#ifndef DAD_TIMER_H_
#define DAD_TIMER_H_

// Config defines
#define SET_TIMER_3_AS_SW_TIMER

// DriverLib Includes
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

// Standard includes
#include <stdint.h>
#include <stdbool.h>

// Timer Defines
#define DAD_DEFAULT_TIMER_TICKS    0x2DC6

// Initialize timer in milliseconds
void DAD_Timer_Initialize_ms(uint16_t period_ms, uint32_t timerBase, Timer_A_UpModeConfig *timerConfig);

// Initialize timer in microseconds
void DAD_Timer_Initialize_us(uint16_t period_us, uint32_t timerBase, Timer_A_UpModeConfig *timerConfig);

// Start Timer
void DAD_Timer_Start(uint32_t timerBase);

// Check wheter timer has finished
bool DAD_Timer_Has_Finished(uint32_t timerBase);

// Stop Timer
    // Returns current time
double DAD_Timer_Stop(uint32_t timerBase, Timer_A_UpModeConfig *timerConfig);

// Return current time
    // Returns time (in us or ms depending on how timer was initialized)
double DAD_Timer_Get_Time(uint32_t timerBase, Timer_A_UpModeConfig *timerConfig);

static void DAD_Timer_Set_Interrupt(uint32_t timerBase);

#endif /* DAD_TIMER_H_ */
