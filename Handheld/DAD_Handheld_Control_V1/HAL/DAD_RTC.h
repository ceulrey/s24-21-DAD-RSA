/*
 * DAD_RTC.h
 *
 *  Created on: Apr 9, 2023
 *      Author: Max
 */

#ifndef DAD_RTC_H_
#define DAD_RTC_H_

// DriverLib Includes
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

// Std includes
#include <stdio.h>

#define FLASH_RW_ADDRESS    0x0003F000
#define FLASH_BLOCK_SIZE    8
#define HAS_BEEN_INIT       12


#define RTC_OUTPUT_STR_LEN 25   // Length of RTC string otput

// Init/start RTC
void DAD_RTC_init(RTC_C_Calendar* calendarStruct);

// Init/start RTC w/o input
void DAD_RTC_initZero();

// Init/start RTC from flash
void DAD_RTC_initFromFlash();

// Save current time to flash
void DAD_RTC_saveTime();

// Return date/time as a string
void DAD_RTC_getTime(char* currentTime);

#endif /* DAD_RTC_H_ */
