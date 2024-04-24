/*
 * DAD_UART.c
 *
 *  Created on: Jan 16, 2023
 *      Author: Maximilien
 */

#include <HAL/DAD_UART.h>

// Declare buffer pointers. Pointers stored directly in RAM for use in ISR
static volatile RingBuf_Handle UART0_BuffPtr;
static volatile RingBuf_Handle UART1_BuffPtr;
static volatile RingBuf_Handle UART2_BuffPtr;
static volatile RingBuf_Handle UART3_BuffPtr;

// Sets the config struct to specific baudrate
void DAD_UART_Set_Config(uint16_t baudRate, uint32_t moduleInstance, DAD_UART_Struct* UARTPtr){

    // Set Baud rate and modulation registers
    double divisionFactor = MAP_CS_getSMCLK() / ((double)baudRate);                     // N
    UARTPtr->uartConfig.selectClockSource =  EUSCI_A_UART_CLOCKSOURCE_SMCLK;            // Can be any freq
    UARTPtr->uartConfig.clockPrescalar = (uint_fast16_t)(divisionFactor/16);            // int(N/16)
    UARTPtr->uartConfig.firstModReg = (uint_fast8_t)((divisionFactor/16-((int)divisionFactor/16))*16);            // first modulator register
    UARTPtr->uartConfig.secondModReg = DAD_UART_Find_Second_Mod_Reg(divisionFactor);    // Set modulation rate
    UARTPtr->uartConfig.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;   // Oversampling

    // Set other UART configuration
    UARTPtr->uartConfig.parity = EUSCI_A_UART_NO_PARITY;
    UARTPtr->uartConfig.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    UARTPtr->uartConfig.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    UARTPtr->uartConfig.dataLength = EUSCI_A_UART_8_BIT_LEN;
    UARTPtr->uartConfig.uartMode = EUSCI_A_UART_MODE;

    // Set Module instance
    UARTPtr->moduleInst = moduleInstance;

    // Debug - Set up LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
}


// Start UART communication
bool DAD_UART_Init(DAD_UART_Struct* UARTPtr, size_t bufferSize){
    // Init and enable UART module
    if(!MAP_UART_initModule(UARTPtr->moduleInst, &UARTPtr->uartConfig))
        return false;
    MAP_UART_enableModule(UARTPtr->moduleInst);

    // Init buffer
    UARTPtr->bufPtr = (char*)malloc(bufferSize * sizeof(char));
    if(UARTPtr->bufPtr == NULL)
        return false;   // Insuff space
    modifiedRingBuf_construct(&(UARTPtr->UART_Buffer), UARTPtr->bufPtr, bufferSize);

    // Choose which interrupt, which pins to set
    uint32_t interruptNum = INT_EUSCIA0;
    switch(UARTPtr->moduleInst){
    case EUSCI_A0_BASE:
        interruptNum = INT_EUSCIA0;                                     // Interrupt for A0
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,    // GPIO for A0 UART
                  GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
        UART0_BuffPtr = &UARTPtr->UART_Buffer;                          // Set BuffPtr
        break;
    case EUSCI_A1_BASE:
        interruptNum = INT_EUSCIA1;                                     // Interrupt for A1
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,    // GPIO for A1 UART
                              GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
        UART1_BuffPtr = &UARTPtr->UART_Buffer;                          // Set BuffPtr
        break;
    case EUSCI_A2_BASE:
        interruptNum = INT_EUSCIA2;                                     // Interrupt for A2
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,    // GPIO for A2 UART
                          GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
        UART2_BuffPtr = &UARTPtr->UART_Buffer;                          // Set BuffPtr
        break;
    case EUSCI_A3_BASE:
        interruptNum = INT_EUSCIA3;                                     // Interrupt for A3
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P9,    // GPIO for A3 UART
                      GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
        UART3_BuffPtr = &UARTPtr->UART_Buffer;                          // Set BuffPtr
        break;
    }

    // Enable Interrupts
    MAP_UART_enableInterrupt(UARTPtr->moduleInst, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(interruptNum);
    MAP_Interrupt_enableMaster();
    return true;
}

// Write single char
void DAD_UART_Write_Char(DAD_UART_Struct* UARTPtr, char c){
    MAP_UART_transmitData(UARTPtr->moduleInst, c);
}

// Write full message
void DAD_UART_Write_Str(DAD_UART_Struct* UARTPtr, char* msg){
    uint16_t msgLength = strlen(msg);
    uint16_t i;
    for(i = 0; i < msgLength; i++){
        MAP_UART_transmitData(UARTPtr->moduleInst, msg[i]);
    }
}

// Stop UART
void DAD_UART_Stop(DAD_UART_Struct* UARTPtr){
    uint16_t interruptNum = 0;
    switch(UARTPtr->moduleInst){
        case EUSCI_A0_BASE:
            interruptNum = INT_EUSCIA0;                                     // Interrupt for A0
            break;
        case EUSCI_A1_BASE:
            interruptNum = INT_EUSCIA1;                                     // Interrupt for A1
            break;
        case EUSCI_A2_BASE:
            interruptNum = INT_EUSCIA2;                                     // Interrupt for A2
            break;
        case EUSCI_A3_BASE:
            interruptNum = INT_EUSCIA3;                                     // Interrupt for A3
            break;
        }

    MAP_UART_disableInterrupt(UARTPtr->moduleInst, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_disableInterrupt(interruptNum);
    MAP_UART_disableModule(UARTPtr->moduleInst);
}



bool DAD_UART_HasChar(DAD_UART_Struct* UARTPtr){
    return modifiedRingBuf_getCount(&(UARTPtr->UART_Buffer)) > 0;
}

char DAD_UART_GetChar(DAD_UART_Struct* UARTPtr){
    char c = '\0';
    modifiedRingBuf_get(&(UARTPtr->UART_Buffer), &c);
    return c;
}

void DAD_UART_GetCharPtr(DAD_UART_Struct* UARTPtr, char* c){
    modifiedRingBuf_get(&(UARTPtr->UART_Buffer), c);
}

void DAD_UART_Peek(DAD_UART_Struct* UARTPtr, char* c){
    modifiedRingBuf_peek(&(UARTPtr->UART_Buffer), c);
}

size_t DAD_UART_NumCharsInBuffer(DAD_UART_Struct* UARTPtr){
    return modifiedRingBuf_getCount(&(UARTPtr->UART_Buffer));
}

// Disable Interrupts
void DAD_UART_DisableInt(DAD_UART_Struct* UARTPtr){
    MAP_UART_disableInterrupt(UARTPtr->moduleInst, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

// Enable Interrupts
void DAD_UART_EnableInt(DAD_UART_Struct* UARTPtr){
    MAP_UART_enableInterrupt(UARTPtr->moduleInst, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

// Returns the value for the second modulation register
    // Uses fractional part of division factor to look through table.
    // Return value just below fractional part
static uint8_t DAD_UART_Find_Second_Mod_Reg(float divisionFactor){
    // lookup table
    float fract = divisionFactor - ((int)divisionFactor);
    float table[] = {0.0000, 0.0529, 0.0715, 0.0835, 0.1001, 0.1252, 0.1430, 0.1670, 0.2147, 0.2224, 0.2503, 0.3000, 0.3335, 0.3575, 0.3753, 0.4003, 0.4286, 0.4378,
                     0.5002, 0.5715, 0.5715, 0.6003, 0.6254, 0.6432, 0.6667, 0.7001, 0.7147, 0.7503, 0.7861, 0.8004, 0.8333, 0.8464, 0.8572, 0.8751, 0.9004, 0.9170, 0.9288, 1};

    // Run through lookup table
    // If fract is 0, return 0
    if(fract == 0)
        return 0x00;
    // If fract is greater than value at i, return value at i-1
    uint8_t i;
    for(i = 1; i < 0xFF; i++){
        if(fract > table[i])
            return i - 1;
    }
    return i;
}



void EUSCIA0_IRQHandler(void){
    uint32_t intStatus = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    if(intStatus & EUSCI_A_UART_RECEIVE_INTERRUPT){
        modifiedRingBuf_put(UART0_BuffPtr, MAP_UART_receiveData(EUSCI_A0_BASE)); // Put received data at end of buffer
    }

    //Clear all interrupts lol
    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, 0xFF);
}

void EUSCIA1_IRQHandler(void){
    uint32_t intStatus = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

    if(intStatus & EUSCI_A_UART_RECEIVE_INTERRUPT){
        modifiedRingBuf_put(UART1_BuffPtr, MAP_UART_receiveData(EUSCI_A1_BASE)); // Put received data at end of buffer
    }

    //Clear all interrupts lol
    MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, 0xFF);
}

void EUSCIA2_IRQHandler(void){
    uint32_t intStatus = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    if(intStatus & EUSCI_A_UART_RECEIVE_INTERRUPT){
        modifiedRingBuf_put(UART2_BuffPtr, MAP_UART_receiveData(EUSCI_A2_BASE)); // Put received data at end of buffer
    }

    //Clear all interrupts lol
    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, 0xFF);
}

void EUSCIA3_IRQHandler(void){
    uint32_t intStatus = MAP_UART_getEnabledInterruptStatus(EUSCI_A3_BASE);

    if(intStatus & EUSCI_A_UART_RECEIVE_INTERRUPT){
        modifiedRingBuf_put(UART3_BuffPtr, MAP_UART_receiveData(EUSCI_A3_BASE)); // Put received data at end of buffer
    }

    //Clear all interrupts lol
    MAP_UART_clearInterruptFlag(EUSCI_A3_BASE, 0xFF);
}
