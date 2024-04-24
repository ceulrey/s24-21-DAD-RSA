/*
 * DAD_RTC.c
 *
 *  Created on: Apr 9, 2023
 *      Author: Max
 */

#include <HAL/DAD_RTC.h>

// Init/start RTC
void DAD_RTC_init(RTC_C_Calendar* calendarStruct){
    MAP_CS_initClockSignal(CS_BCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN0 | GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_CS_setExternalClockSourceFrequency(32000,48000000);
    MAP_CS_startLFXT(CS_LFXT_DRIVE3);

    MAP_RTC_C_initCalendar(calendarStruct, RTC_C_FORMAT_BINARY);
    MAP_RTC_C_startClock();
}

void DAD_RTC_initZero(){
    RTC_C_Calendar calendarStruct = {0, 0, 0, 0, 0, 0, 0};

    calendarStruct.dayOfWeek = 0x00;
    calendarStruct.dayOfmonth = 0x00;
    calendarStruct.hours = 0x00;
    calendarStruct.minutes= 0x00;
    calendarStruct.month = 0x00;
    calendarStruct.seconds = 0x00;
    calendarStruct.year = 000;
    MAP_CS_initClockSignal(CS_BCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN0 | GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_CS_setExternalClockSourceFrequency(32000,48000000);
    MAP_CS_startLFXT(CS_LFXT_DRIVE3);

    MAP_RTC_C_initCalendar(&calendarStruct, RTC_C_FORMAT_BINARY);
    MAP_RTC_C_startClock();
}

void DAD_RTC_initFromFlash(){
    // Flash Setup
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 1);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 1);

    // Read from flash
    uint8_t readFromFlash[FLASH_BLOCK_SIZE];
    int i;
    for(i = 0; i < FLASH_BLOCK_SIZE; i++)
        readFromFlash[i] = *(uint8_t*)(FLASH_RW_ADDRESS + i);

    // Read RTC
    if(readFromFlash[0] == HAS_BEEN_INIT){
        RTC_C_Calendar calendarStruct;
        calendarStruct.seconds = readFromFlash[1];
        calendarStruct.minutes = readFromFlash[2];
        calendarStruct.hours = readFromFlash[3];
        calendarStruct.dayOfmonth = readFromFlash[4];
        calendarStruct.month = readFromFlash[5];
        calendarStruct.year = readFromFlash[6];
        calendarStruct.dayOfWeek = readFromFlash[7];
        DAD_RTC_init(&calendarStruct);
    }
    else{
        DAD_RTC_initZero();
    }
}


// Save current time to flash
void DAD_RTC_saveTime(){
    RTC_C_Calendar calendarStruct = MAP_RTC_C_getCalendarTime();
    uint8_t rtcArr[FLASH_BLOCK_SIZE];
    rtcArr[0] = HAS_BEEN_INIT;
    rtcArr[1] = calendarStruct.seconds;
    rtcArr[2] = calendarStruct.minutes;
    rtcArr[3] = calendarStruct.hours;
    rtcArr[4] = calendarStruct.dayOfmonth;
    rtcArr[5] = calendarStruct.month;
    rtcArr[6] = calendarStruct.year;
    rtcArr[7] = calendarStruct.dayOfWeek;
    MAP_FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1,FLASH_SECTOR31);
    MAP_FlashCtl_eraseSector(FLASH_RW_ADDRESS);
    bool test = MAP_FlashCtl_programMemory(rtcArr, (void*)FLASH_RW_ADDRESS, FLASH_BLOCK_SIZE);
    MAP_FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1,FLASH_SECTOR31);
}


// Return date/time as a string
void DAD_RTC_getTime(char* currentTime){
    RTC_C_Calendar cal = MAP_RTC_C_getCalendarTime();

    // DD/MM/YYYY HR:MIN:SEC
    sprintf(currentTime, "%02d/%02d/%04d %02d:%02d:%02d",
            cal.dayOfmonth, cal.month, cal.year, cal.hours, cal.minutes, cal.seconds);
}
