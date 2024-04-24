/*
 * DAD_GPIO.h
 *
 *  Created on: Mar 31, 2023
 *      Author: Max
 */

#ifndef DAD_GPIO_H_
#define DAD_GPIO_H_

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>

#define NUM_PAGE_PINS   4

typedef struct DAD_GPIO_Pin_{
    uint8_t port;
    uint16_t pin;
}DAD_GPIO_Pin;

typedef struct DAD_GPIO_Struct_{
    DAD_GPIO_Pin currentPagePins[NUM_PAGE_PINS];
    DAD_GPIO_Pin startStopPin;
}DAD_GPIO_Struct;

void DAD_GPIO_Init(DAD_GPIO_Struct* gpioStruct);

uint8_t DAD_GPIO_getPage(DAD_GPIO_Struct* gpioStruct);

bool DAD_GPIO_getStartStop(DAD_GPIO_Struct* gpioStruct);

#endif /* DAD_GPIO_H_ */
