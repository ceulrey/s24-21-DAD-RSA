/*
 * DAD_Calc.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Max
 */

#ifndef DAD_CALC_H_
#define DAD_CALC_H_

// Standard includes
#include <stdlib.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

// Utils
#include <math.h>
#include <stdint.h>

#define MOVING_AVERAGE_N            128    // Calculate the moving averages based on the past N data packets
#define FFT_SIZE                    512
#define PACKET_TYPE_MASK            7
#define NUM_OF_PORTS                8

// TODO remove on integration
typedef enum {TEMP = 0b000, HUM = 0b001, VIB = 0b010, MIC = 0b011, LOWBAT = 0b100, ERR = 0b101, STOP = 0b110, START = 0b111} packetType;

typedef struct DAD_Calc_Struct_{
    float       list[MOVING_AVERAGE_N];
    packetType  type;
    uint16_t    numSamplesCollected;
} DAD_Calc_Struct;

// Initializes struct's values
void DAD_Calc_InitStruct(DAD_Calc_Struct* calcStruct);

// Moving average
    // Takes newest reading, updates sensor's moving average
float DAD_Calc_MovingAvg(uint16_t data, packetType type, DAD_Calc_Struct* calcStruct);

// Average intensity
    // Takes reading, updates sensor's average intensity
float DAD_Calc_AvgIntensity(uint8_t dataSet[FFT_SIZE-2], packetType type);

// Clears moving average data. To be called on sensor disconnect
void DAD_Calc_Clear_History(DAD_Calc_Struct* calcStruct);

// Condition any packet
float DAD_Calc_conditionPacket(uint8_t* packet);

// TODO Condition Humidity data
float DAD_Calc_ConditionHum(uint8_t* packet);

// TODO Condition Temperature Data
float DAD_Calc_ConditionTemp(uint8_t* packet);

// TODO Condition Vib Data
float DAD_Calc_ConditionVib(uint8_t* packet);

// TODO Condition Vib Data
float DAD_Calc_ConditionMic(uint8_t* packet);
#endif /* DAD_CALC_H_ */
