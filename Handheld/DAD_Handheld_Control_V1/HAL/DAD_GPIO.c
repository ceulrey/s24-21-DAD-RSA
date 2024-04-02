/*
 * DAD_GPIO.c
 *
 *  Created on: Mar 31, 2023
 *      Author: Max
 */
#include <HAL/DAD_GPIO.h>

void DAD_GPIO_Init(DAD_GPIO_Struct* gpioStruct){

    gpioStruct->currentPagePins[0].port =   GPIO_PORT_P4;
    gpioStruct->currentPagePins[1].port =   GPIO_PORT_P4;
    gpioStruct->currentPagePins[2].port =   GPIO_PORT_P4;
    gpioStruct->currentPagePins[3].port =   GPIO_PORT_P4;
    gpioStruct->currentPagePins[0].pin =    GPIO_PIN0;
    gpioStruct->currentPagePins[1].pin =    GPIO_PIN1;
    gpioStruct->currentPagePins[2].pin =    GPIO_PIN2;
    gpioStruct->currentPagePins[3].pin =    GPIO_PIN3;


    gpioStruct->startStopPin.port =     GPIO_PORT_P5;
    gpioStruct->startStopPin.pin =      GPIO_PIN5;

    int pin;
    for(pin = 0; pin < NUM_PAGE_PINS; pin++){
        MAP_GPIO_setAsInputPinWithPullUpResistor(gpioStruct->currentPagePins[pin].port, gpioStruct->currentPagePins[pin].pin);
    }
    MAP_GPIO_setAsInputPinWithPullUpResistor(gpioStruct->startStopPin.port, gpioStruct->startStopPin.pin);
}


// Uses pins 0 through 3, with 3 as MSB
uint8_t DAD_GPIO_getPage(DAD_GPIO_Struct* gpioStruct){
    uint8_t output = 0;
    uint8_t pin;
    for(pin = 0; pin < NUM_PAGE_PINS; pin++){
        uint8_t debugVal = (MAP_GPIO_getInputPinValue(gpioStruct->currentPagePins[pin].port, gpioStruct->currentPagePins[pin].pin) << pin);
        output = output | (MAP_GPIO_getInputPinValue(gpioStruct->currentPagePins[pin].port, gpioStruct->currentPagePins[pin].pin) << pin);
    }
    return output;
}

bool DAD_GPIO_getStartStop(DAD_GPIO_Struct* gpioStruct){
    return MAP_GPIO_getInputPinValue(gpioStruct->startStopPin.port, gpioStruct->startStopPin.pin) == 1;
}
