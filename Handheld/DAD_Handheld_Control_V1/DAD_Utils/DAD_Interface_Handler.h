/*
 * DAD_Packet_Handler.h
 *
 *  Created on: Mar 9, 2023
 *      Author: Will Katz, Max Engel

    Meant to abstract the interfaces away.

 */

#ifndef DAD_INTERFACE_HANDLER_H_
#define DAD_INTERFACE_HANDLER_H_

// Standard includes
#include <stdio.h>
#include <stdlib.h>

// Utils
#include <stdint.h>
#include <string.h>
#include <DAD_Utils/DAD_LUTs.h>
#include <DAD_Utils/DAD_Calc.h>

// HAL Includes
#include <HAL/DAD_UART.h>
#include <HAL/DAD_microSD.h>
#include <HAL/DAD_GPIO.h>
#include <HAL/DAD_SW_Timer.h>
#include <HAL/DAD_RTC.h>

// Configuration macros
// #define LOG_INPUT
// #define REPORT_FAILURE
#define WRITE_TO_ONLY_ONE_FILE
#define WRITE_TO_HMI
#define WRITE_TO_MICRO_SD
#define GET_GPIO_FEEDBACK
#define AVG_INTENSITY

// UART Macros
#define RSA_BAUD            57600
#define RSA_BUFFER_SIZE     700
#define HMI_BAUD            38400
#define HMI_BUFFER_SIZE     1
#define MAX_FILENAME_SIZE   12
#ifndef PORT_2_2_AS_RSA
#define RSA_RX_UART_HANDLE  EUSCI_A0_BASE
#else
#define RSA_RX_UART_HANDLE  EUSCI_A1_BASE
#endif
#define HMI_TX_UART_HANDLE  EUSCI_A2_BASE
#define HMI_RX_UART_HANDLE  EUSCI_A3_BASE
#define HMI_THROTTLE_PERIOD_MS  1000

// FSM Timer Macros
#define FSM_TIMER_HANDLE TIMER_A0_BASE
#define FSM_TIMER_PERIOD    750                             // Period in ms. Triggers an interrupt to kick off the FSM every so often.

// Packet Macros
#define STATUS_MASK         24
#define PACKET_TYPE_MASK    7
#define PORT_MASK           224
#define PACKET_SIZE         4                               // Excludes end char 255
#define MESSAGE_LEN         37
#define NUM_OF_PORTS        8                               // Number of ports
#define SIZE_OF_FFT         512
#define HMI_MSG_TYPE_MASK   0b11000000                      // Masks for what type each message is
#define HMI_MSG_DATA_MASK   0b00111111
#define HMI_MSG_START_CMD   254
#define HMI_MSG_STOP_CMD    255


typedef enum {DISCON, CON_D, CON_ND, MSG} packetStatus;
typedef enum {HOUR = 0, MIN = 1, SEC = 3, OTHER = 4} HMI_msgType;
typedef enum {RED = 0b1111100000000000, BLUE = 0b0000000000011111, BLACK = 0b0000000000000000} HMI_color;

// Structure for requesting FFT updates to UI.
typedef struct FFTstruct_{
    bool requestedWriteToUI;
    packetType type;
} FFTstruct;


// Structure for encapsulating hardware/utility interaction
    // Intended to be implemented as a singleton
typedef struct DAD_Interface_Struct_{
    // UART
    DAD_UART_Struct RSA_UART_struct;
    DAD_UART_Struct HMI_TX_UART_struct;
    DAD_UART_Struct HMI_RX_UART_struct;
    DAD_UART_Struct microSD_UART;

    // Timer Control
    Timer_A_UpModeConfig FSMtimerConfig;
    char RTC_currentTime[22];

    // For writing to periphs
    uint8_t sensorPortOrigin;                   // Describes where the package is coming from
    char fileName[MAX_FILENAME_SIZE + 1];       // File name

    // HMI
    DAD_GPIO_Struct gpioStruct;
    uint8_t         currentHMIPage;
    bool            startStop;                  // True when start

    // Utils
    DAD_Calc_Struct tempCalcStruct[NUM_OF_PORTS];   // TODO organize this into a substruct
    DAD_Calc_Struct humCalcStruct[NUM_OF_PORTS];
    uint64_t        timeOfLastFFTSent[NUM_OF_PORTS];
    DAD_LUT_Struct  lutStruct;
    uint64_t lastConnectedTime_ms;

} DAD_Interface_Struct;

// Initializes UART, timers necessary
void DAD_initInterfaces(DAD_Interface_Struct* interfaceStruct);

// Build packet from data in UART buffer
bool DAD_constructPacket(uint8_t* packet, DAD_UART_Struct* UARTptr);

// Write single packet of Temp or HUM to HMI
void DAD_writeSlowDataToUI(uint16_t data, packetType type, DAD_Interface_Struct* interfaceStruct);

// Writes single packet of data to microSD
void DAD_writeSlowDataToMicroSD(uint16_t data, packetType type, DAD_Interface_Struct* interfaceStruct);

// Write moving average for Freq or Temp
void DAD_writeMovingAvgToUI(uint16_t data, packetType type, DAD_Interface_Struct* interfaceStruct);

// Add packet to frequency buffer
bool DAD_addToFreqBuffer(uint8_t packet[PACKET_SIZE+1], DAD_Interface_Struct* interfaceStruct);

// Write frequency data to UI and microSD
void DAD_writeFreqToPeriphs(packetType type, DAD_Interface_Struct* interfaceStruct);

// Checks whether type needs FFT, tells HMI whether to expect FFT
void DAD_Tell_UI_Whether_To_Expect_FFT(packetType type, DAD_Interface_Struct* interfaceStruct);

// Display avg intensity of vibration type
void DAD_displayAvgIntensity(packetType type, DAD_Interface_Struct* interfaceStruct);

// Find out which FFT to run
void DAD_handle_UI_Feedback(DAD_Interface_Struct* interfaceStruct);

// Write Commands to UI (error messages, etc)
void DAD_writeCMDToUI(char* msg, HMI_color color, DAD_Interface_Struct* interfaceStruct);

#ifdef LOG_INPUT
void DAD_logDebug(uint8_t* packet, DAD_Interface_Struct* interfaceStruct);
#endif

#endif /* DAD_INTERFACE_HANDLER_H_ */
