// S23, Team 42
// NAVAIR Data Acquisition Device
// Maximilien Engel

/*
 * Role:    Takes in data from bluetooth over UART, buffers it and spits it out along with relevant metrics to both GUI and microSD
 *
 * Note:    HMI and GUI are sometimes used interchangeably.
 *
 */

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// HAL includes
#include <HAL/DAD_UART.h>

// Control software includes
#include <DAD_Utils/DAD_Interface_Handler.h>
#include <DAD_FSM.h>

// Unfulfilled Requirements
    // UI to MCU config storage
    // true-to-life RTC
    // Data conditioning

// Unfulfilled Quality of life
    // ensure singleton
    // low power shutdown
    // raise the alarm when sensor hasn't said anything in a while

int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();

    // Declare hardware interface struct
    DAD_Interface_Struct interfaceStruct;

    // Application loop
    FSMstate state = STARTUP;
    while(true){
        DAD_FSM_control(&state, &interfaceStruct);              // Handle everything
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);    // Debug - check that it's not hung up
        MAP_PCM_gotoLPM0();                                     // Go back to sleep until next interrupt
    }
}
