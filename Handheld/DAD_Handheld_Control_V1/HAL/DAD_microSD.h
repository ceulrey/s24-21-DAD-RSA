/*
 * DAD_microSD.h
 *
 *  Created on: Feb 19, 2023
 *      Author: Max
 */

#ifndef HAL_DAD_MICROSD_H_
#define HAL_DAD_MICROSD_H_

//Standard includes
#include <stdlib.h>
#include <stdbool.h>

#include <string.h>
#include <HAL/DAD_UART.h>
#include <HAL/DAD_Timer.h>

// Config macros
// #define PORT_2_2_AS_RSA

#define MICRO_SD_TIMER_MODULE   TIMER_A2_BASE
#define MICRO_SD_CMD_DELAY      50                   // Wait until command mode is entered
#define MICRO_SD_BAUD_RATE      57600
#define MICRO_SD_BUFF_SIZE      1

#ifndef PORT_2_2_AS_RSA
#define MICRO_SD_MODULE_INSTANCE EUSCI_A1_BASE  // Module instance - module A2 for debug
#else
#define MICRO_SD_MODULE_INSTANCE EUSCI_A3_BASE  // Module instance
#endif


// Enter command mode
static void DAD_microSD_enterCMD(DAD_UART_Struct* uartStruct);

// Configures and Initializes UART
    // Parameter - structure for using the UART HAL
bool DAD_microSD_InitUART(DAD_UART_Struct* uartStruct);

// Opens file
    // Step into cmd mode, append to file
    // If filename does not exist, file is created
    // Parameter - array of chars for file name. 8.3 format
    // Parameter - structure for using the UART HAL
bool DAD_microSD_openFile(char* fileName, DAD_UART_Struct* uartStruct);

// Write to file
    // Parameter - array of chars for file name. 8.3 format
    // Parameter - array of strs for message. writes as CSV
    // Parameter - structure for using the UART HALs
bool DAD_microSD_Write_CSV(char* fileName, char** message, uint16_t messageLen, DAD_UART_Struct* uartStruct);

void DAD_microSD_Write(char* message, DAD_UART_Struct* uartStruct);

#endif /* HAL_DAD_MICROSD_H_ */
