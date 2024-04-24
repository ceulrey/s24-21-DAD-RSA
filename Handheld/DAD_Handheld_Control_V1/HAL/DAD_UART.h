/*
 * DAD_UART.h
 *
 *  Created on: Jan 16, 2023
 *      Author: maxim
 */

/*
 // Example usage:

  eUSCI_UART_ConfigV1 uartConfig;
     DAD_UART_Set_Config(9600, &uartConfig);

     // initialize and enable EUSCI_A0
     DAD_UART_Init(EUSCI_A0_BASE, &uartConfig);

     while(true){
         DAD_UART_Write_Test(EUSCI_A0_BASE, 'P');
     }

 */


//Intended as an abstraction to the UART driver


#ifndef DAD_UART_H_
#define DAD_UART_H_

// Driverlib
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

// Standard Includes
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Specific to UART HAL
#include <ti/drivers/UART.h>            // Import the UART driver definitions
#include <DAD_Utils/modifiedRingbuf.h>            // Import ring buffer

typedef struct DAD_UART_Struct_{
    char* bufPtr;
    eUSCI_UART_ConfigV1 uartConfig;
    uint32_t moduleInst;
    RingBuf_Object UART_Buffer;
} DAD_UART_Struct;

// Populates the config, sets the baud rate
void DAD_UART_Set_Config(uint16_t baudRate, uint32_t moduleInstance, DAD_UART_Struct* UARTPtr);

// Initializes the uart module with the specified config information. Needs a buffer to put characters in.
bool DAD_UART_Init(DAD_UART_Struct* UARTPtr, size_t bufferSize);

// Stop UART
void DAD_UART_Stop(DAD_UART_Struct* UARTPtr);

// Get single char from UART
char DAD_UART_GetChar(DAD_UART_Struct* UARTPtr);

// Get char from buffer, assign pointer of c to char
void DAD_UART_GetCharPtr(DAD_UART_Struct* UARTPtr, char* c);

// Write a single char to microSD
void DAD_UART_Write_Char(DAD_UART_Struct* UARTPtr, char c);

// Peek char at front of buffer
void DAD_UART_Peek(DAD_UART_Struct* UARTPtr, char* c);

// Write a string to microSD
void DAD_UART_Write_Str(DAD_UART_Struct* UARTPtr, char* msg);

// At least 1 char is ready
bool DAD_UART_HasChar(DAD_UART_Struct* UARTPtr);

// Disable Interrupts
void DAD_UART_DisableInt(DAD_UART_Struct* UARTPtr);

// Enable Interrupts
void DAD_UART_EnableInt(DAD_UART_Struct* UARTPtr);

// Returns number of chars in buffer
size_t DAD_UART_NumCharsInBuffer(DAD_UART_Struct* UARTPtr);

// Used in Set_Config
    //Returns the value for the second modulation register
    // Uses fractional part of division factor to look through table.
    // Return value just below fractional part
static uint8_t DAD_UART_Find_Second_Mod_Reg(float divisionFactor);

#endif /* DAD_UART_H_ */
