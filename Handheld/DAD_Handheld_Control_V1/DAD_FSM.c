/*
 * DAD_FSM.c
 *
 *  Created on: Mar 9, 2023
 *      Author: Max Engel
 */

#include <DAD_FSM.h>
#include <DAD_Packet_Handler.h>

static uint64_t lastDataTransmission; // Record previous time of data

void DAD_FSM_control(FSMstate *state, DAD_Interface_Struct* interfaceStruct){
    // FSM
    switch (*state){

    case STARTUP:
        DAD_initInterfaces(interfaceStruct);        // Initialize hardware interfaces necessary for FSM use
        handleMessage(START, interfaceStruct);      // Write Start Message
        *state = RSA_READ;
        break;

    case RSA_READ:

        // When timer expires, start writing to HMI and microSD
        // or
        // Wait for buffer to fill
        if((DAD_Timer_Has_Finished(FSM_TIMER_HANDLE) && DAD_UART_NumCharsInBuffer(&interfaceStruct->RSA_UART_struct) >= MIN_PACKETS_TO_PROCESS)
                || RSA_BUFFER_SIZE ==  DAD_UART_NumCharsInBuffer(&interfaceStruct->RSA_UART_struct)){
            *state = HANDLE_PERIPH;
            lastDataTransmission = interfaceStruct->lastConnectedTime_ms;
        }

        // Check for RSA timeout
        else if(DAD_SW_Timer_getMS(&interfaceStruct->lastConnectedTime_ms)
                && interfaceStruct->lastConnectedTime_ms - lastDataTransmission > RSA_RX_TIMEOUT_PERIOD_MS){
            *state = HANDLE_RSA_TIMEOUT;
        }

        // Check for stop input from user
        else if(!interfaceStruct->startStop){
            *state = STOP_STATE;
        }
        break;
    case HANDLE_PERIPH:
        // Disable RSA UART rx interrupts, ignore all UART input until buffer empty
        DAD_UART_DisableInt(&interfaceStruct->RSA_UART_struct);
        DAD_UART_DisableInt(&interfaceStruct->HMI_RX_UART_struct);
        DAD_Timer_Stop(FSM_TIMER_HANDLE, &interfaceStruct->FSMtimerConfig);

        // Handle all data in the RSA rx buffer
        handleRSABuffer(interfaceStruct);

        // Finished writing to HMI/microSD, start listening again
        *state = RSA_READ;
        DAD_UART_EnableInt(&interfaceStruct->RSA_UART_struct);
        DAD_UART_EnableInt(&interfaceStruct->HMI_RX_UART_struct);

        // Restart Control Timer
        DAD_Timer_Start(FSM_TIMER_HANDLE);

        // Reset timeout timer
        DAD_SW_Timer_getMS(&interfaceStruct->lastConnectedTime_ms);
        break;
    case STOP_STATE:
        if(DAD_Timer_Has_Finished(FSM_TIMER_HANDLE) && DAD_UART_NumCharsInBuffer(&interfaceStruct->RSA_UART_struct) >= MIN_PACKETS_TO_PROCESS){
            handleStop(interfaceStruct);
            DAD_SW_Timer_getMS(&interfaceStruct->lastConnectedTime_ms);     // Reset timeout timer
        }

        if(interfaceStruct->startStop){
           *state = RSA_READ;
        }

        // Restart control timer
        DAD_Timer_Start(FSM_TIMER_HANDLE);
        break;
    case HANDLE_RSA_TIMEOUT:
        handleMessage(ERR, interfaceStruct);

        if(DAD_UART_NumCharsInBuffer(&interfaceStruct->RSA_UART_struct) > MIN_PACKETS_TO_PROCESS){
            handleMessage(START, interfaceStruct);
            *state = RSA_READ;
        }
    }
}
