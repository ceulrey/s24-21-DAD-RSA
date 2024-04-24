/*
 * DAD_Packet_Handler.c
 *
 *  Created on: Mar 25, 2023
 *      Author: Max
 */

#include <DAD_Packet_Handler.h>

void handleRSABuffer(DAD_Interface_Struct* interfaceStruct){
    // Handle RT data
    DAD_RTC_getTime(interfaceStruct->RTC_currentTime);
    DAD_RTC_saveTime();

    // Set GPIO flags for use in this run
    DAD_handle_UI_Feedback(interfaceStruct);

    // Write everything to log
    #ifdef WRITE_TO_ONLY_ONE_FILE
    strcpy(interfaceStruct->fileName, "log.txt");
    DAD_microSD_Write("\nRunning Time: ", &interfaceStruct->microSD_UART);
    DAD_microSD_Write(interfaceStruct->RTC_currentTime, &interfaceStruct->microSD_UART);
    DAD_microSD_openFile(interfaceStruct->fileName, &interfaceStruct->microSD_UART);
    DAD_microSD_Write("\n", &interfaceStruct->microSD_UART);
    #endif

    // Read individual packets from buffer
    while(PACKET_SIZE < DAD_UART_NumCharsInBuffer(&interfaceStruct->RSA_UART_struct)){
        handlePacket(interfaceStruct);
    }

    // Flush out remaining chars from read buffer.
    while(DAD_UART_NumCharsInBuffer(&interfaceStruct->RSA_UART_struct) > 0){
        DAD_UART_GetChar(&interfaceStruct->RSA_UART_struct);
    }

    #ifdef LOG_INPUT
    char* message = "EndBuf\n\n";
    DAD_UART_Write_Str(&interfaceStruct->microSD_UART, message);
    #endif

    // Set GPIO flags for state change
    DAD_handle_UI_Feedback(interfaceStruct);
}

static void handlePacket(DAD_Interface_Struct* interfaceStruct)
{
    // Construct packet
    uint8_t packet[PACKET_SIZE + 1];
    if(!DAD_constructPacket(packet, &interfaceStruct->RSA_UART_struct)){                                // If construct packet fails
        #ifdef REPORT_FAILURE
        // Go to log file
        strcpy(interfaceStruct->fileName, "log.txt");
        DAD_microSD_openFile(interfaceStruct->fileName, &interfaceStruct->microSD_UART);
        interfaceStruct->sensorPortOrigin = 255;

        // Log error to microSD.
        DAD_microSD_Write("Error: Failed to construct packet\n", &interfaceStruct->microSD_UART);   // A packet, misread, could have been.
        #endif
        return;
    }


    #ifdef LOG_INPUT
    // Debug - Log Packet
    DAD_logDebug(packet, interfaceStruct);
    #endif

    // Interpret packet
    packetStatus PKstatus = (packetStatus)((packet[0] & STATUS_MASK) >> 3);
    packetType type = (packetType)(packet[0] & PACKET_TYPE_MASK);
    uint8_t port = (packet[0] & PORT_MASK) >> 5;

    #ifdef WRITE_TO_ONLY_ONE_FILE
    interfaceStruct->sensorPortOrigin = port;
    #endif

    // Deal with packet
    switch(PKstatus)
    {
        case DISCON:
            handleDisconnect(port, type, interfaceStruct);
            break;
        case CON_D:
            handleData(port, type, packet, interfaceStruct);
            break;
        case CON_ND:
            handle_CON_ND(type, interfaceStruct);
            break;
        case MSG:
            handleMessage(type, interfaceStruct);
            break;
        default:
            DAD_microSD_Write("bad packet\n", &interfaceStruct->microSD_UART);
    }
}


static void handleDisconnect(uint8_t port, packetType type, DAD_Interface_Struct* interfaceStruct)
{
    DAD_Tell_UI_Whether_To_Expect_FFT(STOP, interfaceStruct);
    #ifdef WRITE_TO_HMI
    // Write to HMI
    // Report sensor disconnected to HMI
    DAD_UART_Write_Str(&interfaceStruct->HMI_TX_UART_struct, "HOME.s");
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, interfaceStruct->sensorPortOrigin + 49);
    DAD_UART_Write_Str(&interfaceStruct->HMI_TX_UART_struct, "Val.txt=\"NONE\"");
    // End of transmission (1st sensor)
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, 255);
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, 255);
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, 255);
    // Report second sensor disconnected to HMI
    DAD_UART_Write_Str(&interfaceStruct->HMI_TX_UART_struct, "HOME.s");
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, interfaceStruct->sensorPortOrigin + 49);
    DAD_UART_Write_Str(&interfaceStruct->HMI_TX_UART_struct, "Val2.txt=\"\"");
    // End of transmission (2st sensor)
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, 255);
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, 255);
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, 255);

    // Report Stop Sending FFT
    DAD_UART_Write_Str(&interfaceStruct->HMI_TX_UART_struct, "HOME.f");
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, interfaceStruct->sensorPortOrigin + 49);
    DAD_UART_Write_Str(&interfaceStruct->HMI_TX_UART_struct, ".val=0");
    // End of transmission
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, 255);
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, 255);
    DAD_UART_Write_Char(&interfaceStruct->HMI_TX_UART_struct, 255);
    #endif

    // Write to microSD
    // Construct message
    char message[17];
    switch(type){
    case TEMP:
        sprintf(message, "Temp disc @p%d\n", port);
        break;
    case HUM:
        sprintf(message, "Hum disc @p%d\n", port);
        break;
    case VIB:
        sprintf(message, "Vib disc @p%d\n", port);
        break;
    case MIC:
        sprintf(message, "Mic disc @p%d\n", port);
        break;
    default:
        sprintf(message, "Disconnect Error @p%d\n", port);
    }
    // Write to data file
    DAD_microSD_Write(message, &interfaceStruct->microSD_UART);
    // Open the log file
    strcpy(interfaceStruct->fileName, "log.txt");
    DAD_microSD_openFile(interfaceStruct->fileName, &interfaceStruct->microSD_UART);
    // Write to log
    DAD_microSD_Write(message, &interfaceStruct->microSD_UART);
    interfaceStruct->sensorPortOrigin = 255; // Record that current file is log.txt

}


#ifndef THROTTLE_UI_OUTPUT
// Processes data packet, sends data to peripherals
static void handleData(uint8_t port, packetType type, uint8_t packet[PACKET_SIZE], DAD_Interface_Struct* interfaceStruct){

    #ifndef WRITE_TO_ONLY_ONE_FILE                       // Disables writing to multiple files
    // Check that we are writing to the right file
    if(interfaceStruct->sensorPortOrigin != port ){           // Compare data type to the file that is currently being written to
        // Set port
        sprintf(interfaceStruct->fileName, "port%d.csv", port+1);
        interfaceStruct->sensorPortOrigin = port;

        // Open the correct file
        DAD_microSD_openFile(interfaceStruct->fileName, &interfaceStruct->microSD_UART);
    }
    #endif


    // Write data to periphs
    switch(type)
    {
        uint16_t data;

        case TEMP:  // Fall through to HUM
        case HUM:
            data = ((packet[1] << 8) + packet[2]) % 110;
            DAD_writeSlowDataToUI(data, type, interfaceStruct);
            DAD_writeMovingAvgToUI(data, type, interfaceStruct);
            DAD_writeSlowDataToMicroSD(data, type, interfaceStruct);
            break;
        case VIB:   // Fall through to mic. Same code
        case MIC:
            // Add packet to buffer,
            DAD_addToFreqBuffer(packet, interfaceStruct);
            // If second to last packet has been received, write to peripherals
            uint64_t currentTime;
            DAD_SW_Timer_getMS(&currentTime);

            if(packet[1]*2 >= SIZE_OF_FFT - 8
                    && currentTime - interfaceStruct->timeOfLastFFTSent[port] > HMI_THROTTLE_PERIOD_MS){              // Note - second to last packet bc "last packet" would require receiving a byte of 0xFF, which would result in an invalid packet
                DAD_writeFreqToPeriphs(type, interfaceStruct);
                // Record last time of fft sent
                DAD_SW_Timer_getMS(&interfaceStruct->timeOfLastFFTSent[port]);
            }
            break;
    }
}
#endif

// Handles packets of "connected, no data" type
static void handle_CON_ND(packetType type, DAD_Interface_Struct* interfaceStruct){
    DAD_Tell_UI_Whether_To_Expect_FFT(type, interfaceStruct);
}

void handleMessage(packetType type, DAD_Interface_Struct* interfaceStruct){
    // Open the microSD log file
    strcpy(interfaceStruct->fileName, "log.txt");
    DAD_microSD_openFile(interfaceStruct->fileName, &interfaceStruct->microSD_UART);
    interfaceStruct->sensorPortOrigin = 255;

    char message[50];
    HMI_color color;
    switch(type){
        case LOWBAT:
            sprintf(message, "RSA Low Battery detected");
            color = RED;
            break;
        case ERR:
            sprintf(message, "Error: RSA Error reported");
            color = RED;
            break;
        case STOP:
            sprintf(message, "Data collection halted");
            color = BLUE;
            break;
        case START:
            sprintf(message, "Data collection started");
            color = BLUE;
            break;
        default:
            sprintf(message, "Message Line");
            color = BLUE;
    }
    DAD_microSD_Write(message, &interfaceStruct->microSD_UART);
    DAD_microSD_Write("\n", &interfaceStruct->microSD_UART);
    DAD_writeCMDToUI(message, color, interfaceStruct);
}

// Handles incoming packets when stop is asserted
void handleStop(DAD_Interface_Struct* interfaceStruct){
    #ifdef GET_GPIO_FEEDBACK

    uint8_t packet[PACKET_SIZE + 1];
    packetStatus PKstatus;
    packetType type;
    uint8_t port;

    // Read individual packets from buffer
    while(PACKET_SIZE < DAD_UART_NumCharsInBuffer(&interfaceStruct->RSA_UART_struct)){
        // Construct packet
        if(!DAD_constructPacket(packet, &interfaceStruct->RSA_UART_struct)){                                // If construct packet fails
            #ifdef REPORT_FAILURE
            // Go to log file
            strcpy(interfaceStruct->fileName, "log.txt");
            DAD_microSD_openFile(interfaceStruct->fileName, &interfaceStruct->microSD_UART);
            interfaceStruct->sensorPortOrigin = 255;

            // Log error to microSD.
            DAD_microSD_Write("Error: Failed to construct packet\n", &interfaceStruct->microSD_UART);   // A packet, misread, could have been.
            #endif
            return;
        }
        #ifdef LOG_INPUT
        // Debug - Log Packet
        DAD_logDebug(packet, interfaceStruct);
        #endif

        // Interpret packet
        PKstatus = (packetStatus)((packet[0] & STATUS_MASK) >> 3);
        type = (packetType)(packet[0] & PACKET_TYPE_MASK);
        port = (packet[0] & PORT_MASK) >> 5;

        #ifdef WRITE_TO_ONLY_ONE_FILE
        interfaceStruct->sensorPortOrigin = port;
        #endif

        // Deal with packet
        switch(PKstatus)
        {
            case DISCON:
                handleDisconnect(port, type, interfaceStruct);
                break;
            case CON_D:
                // Throw out data.
                break;
            case CON_ND:
                handle_CON_ND(type, interfaceStruct);
                break;
            case MSG:
                handleMessage(type, interfaceStruct);
                break;
            default:
                DAD_microSD_Write("bad packet\n", &interfaceStruct->microSD_UART);
        }
    }

    // Flush out remaining chars from read buffer.
    while(DAD_UART_NumCharsInBuffer(&interfaceStruct->RSA_UART_struct) > 0){
        DAD_UART_GetChar(&interfaceStruct->RSA_UART_struct);
    }

    // Update GPIO flags
    DAD_handle_UI_Feedback(interfaceStruct);
    #endif
}


