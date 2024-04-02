/*
 * DAD_microSD.c
 *
 *  Created on: Feb 19, 2023
 *      Author: Max
 */

#include <HAL/DAD_microSD.h>

static void DAD_microSD_enterCMD(DAD_UART_Struct* uartStruct){
    // Open openLog microSD controller in cmd mode
    DAD_UART_Write_Str(uartStruct, "&&&&&");
    DAD_UART_Write_Char(uartStruct, 13);        // Carriage return

    // blocks until cmd mode was entered
    //while(DAD_UART_GetChar(uartStruct) != '>');
    Timer_A_UpModeConfig timerCfg;
    DAD_Timer_Initialize_us(50, TIMER_A2_BASE, &timerCfg);
    DAD_Timer_Start(TIMER_A2_BASE);
    while(!DAD_Timer_Has_Finished(TIMER_A2_BASE));
}

bool DAD_microSD_InitUART(DAD_UART_Struct* uartStruct){
    // Initialize UART
    DAD_UART_Set_Config(MICRO_SD_BAUD_RATE, MICRO_SD_MODULE_INSTANCE, uartStruct);  // Config UART
    if(!DAD_UART_Init(uartStruct, MICRO_SD_BUFF_SIZE))                              // Init UART
        while(1);
    //return false;
    MAP_UART_disableInterrupt(uartStruct->moduleInst, EUSCI_A_UART_RECEIVE_INTERRUPT);

    DAD_microSD_enterCMD(uartStruct);

    // Initialize microSD reader
    DAD_UART_Write_Str(uartStruct, "init");
    DAD_UART_Write_Char(uartStruct, 13);        // Carriage return
    return true;
}

bool DAD_microSD_openFile(char* fileName, DAD_UART_Struct* uartStruct){
    // Check name size
    if(strlen(fileName) > 12)
        return false;

    DAD_microSD_enterCMD(uartStruct);

    // Append to file
    char command[20] = "append ";
    strcat(command, fileName);                  // concat fileName into command
    DAD_UART_Write_Str(uartStruct, command);    // Call command to begin writing to fileName
    DAD_UART_Write_Char(uartStruct, 13);        // Carriage return

    //MAP_UART_enableInterrupt(uartStruct->moduleInst, EUSCI_A_UART_RECEIVE_INTERRUPT);

    return true;
}

bool DAD_microSD_Write_CSV(char* fileName, char** message, uint16_t messageLen, DAD_UART_Struct* uartStruct){
    // Check that file was opened
    if(DAD_microSD_openFile(fileName, uartStruct)){
        // Write message
        uint16_t i;
        for(i = 0; i < messageLen; i++){
            DAD_UART_Write_Str(uartStruct, message[i]);
            DAD_UART_Write_Char(uartStruct, ',');           // Separate with commas
        }
        DAD_UART_Write_Char(uartStruct, 13);                // Carriage return
        return true;
    }

    return false;
}

void DAD_microSD_Write(char* message, DAD_UART_Struct* uartStruct){
    DAD_UART_Write_Str(uartStruct, message);
}
