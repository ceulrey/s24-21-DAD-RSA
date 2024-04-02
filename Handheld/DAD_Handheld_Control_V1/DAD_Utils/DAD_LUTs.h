/*
 * DAD_Utils.h
 *
 *  Created on: Mar 20, 2023
 *      Author: Max
 */

#ifndef TARGETCONFIGS_DAD_UTILS_H_
#define TARGETCONFIGS_DAD_UTILS_H_

// Standard includes
#include <stdio.h>
#include <stdlib.h>

// Utils includes
#include <string.h>
#include <stdint.h>

// Macros includes
#define WRITE_TO_HMI_FAST_FFTS

#define FREQ_LUT_SIZE           512
#define MICROSD_LUT_WORD_SIZE   7
#define HMI_LUT_WORD_SIZE       25
#define HMI_FFT_ID              10
#define NUM_OF_PORTS            8
#define SIZE_OF_FFT             512

typedef struct DAD_LUT_Struct_{
    // Lookup tables
    char microSDFreqLUT[FREQ_LUT_SIZE][MICROSD_LUT_WORD_SIZE];
    char HMIFreqLUT[FREQ_LUT_SIZE][HMI_LUT_WORD_SIZE];

    // Buffer pointers for buffering sound/vibration data
        // Buffers are loaded up with data.
        // Data from buffer is then sent all at once
    uint8_t freqBuf [NUM_OF_PORTS][SIZE_OF_FFT];
} DAD_LUT_Struct;

// Initialize frequency Lookup Table
    // Lookup table connects char to str output
    // avoids having to go through sprintf every time
void DAD_Utils_initFreqLUT(DAD_LUT_Struct* utilsStruct);

// Using given uint8_t, return corresponding string to write to microSD
char* DAD_Utils_getMicroSDStr(uint8_t num, DAD_LUT_Struct* utilsStruct);

// Using given uint8_t, return corresponding string to write to HMI
char* DAD_Utils_getHMIStr(uint8_t num, DAD_LUT_Struct* utilsStruct);

#endif /* TARGETCONFIGS_DAD_UTILS_H_ */
