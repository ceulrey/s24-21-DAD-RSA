/*
 * DAD_Timer.c
 *
 *  Created on: Dec 31, 2022
 *      Author: Maximilien Engel
 */

#include <HAL/DAD_Timer.h>

// Timer Variables
static volatile bool DAD_timerHasExpired0 = true;
static volatile bool DAD_timerHasExpired1 = true;
static volatile bool DAD_timerHasExpired2 = true;
static volatile bool DAD_timerHasExpired3 = true;

// Initialize timer with default function
void DAD_Timer_Initialize_ms(uint16_t period_ms, uint32_t timerBase, Timer_A_UpModeConfig *timerConfig){
    // Stop timer if in use
    MAP_Timer_A_stopTimer(timerBase);

    // Set up LED (DEBUG)
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    // Set config
    timerConfig->clockSource = TIMER_A_CLOCKSOURCE_ACLK;                                        // ACLK Clock Source
    timerConfig->clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_32;                           // 1024Hz =~1ms period
    timerConfig->timerPeriod = (uint_fast16_t)(0.9765625f * period_ms);                         // Period in ticks
    timerConfig->timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;                    // Disable Timer interrupt
    timerConfig->captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;  // Enable CCR0 interrupt
    timerConfig->timerClear = TIMER_A_DO_CLEAR;                                                 // Clear value

    //Configure timer mode
    MAP_Timer_A_configureUpMode(timerBase, timerConfig);
}

void DAD_Timer_Initialize_us(uint16_t period_us, uint32_t timerBase, Timer_A_UpModeConfig *timerConfig){
    // Stop timer if in use
    MAP_Timer_A_stopTimer(timerBase);

    // Set up LED (DEBUG)
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    // Set config
    float freq = MAP_CS_getSMCLK()/1000000.0;
    timerConfig->clockSource = TIMER_A_CLOCKSOURCE_SMCLK;                                       // ACLK Clock Source
    timerConfig->clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;                            // 1024Hz =~1ms period
    timerConfig->timerPeriod = (uint_fast16_t)(period_us * freq);                               // Period in ticks
    timerConfig->timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;                    // Disable Timer interrupt
    timerConfig->captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;  // Enable CCR0 interrupt
    timerConfig->timerClear = TIMER_A_DO_CLEAR;                                                 // Clear value

    //Configure timer mode
    MAP_Timer_A_configureUpMode(timerBase, timerConfig);
}

//Start Timer
void DAD_Timer_Start(uint32_t timerBase){
    DAD_Timer_Set_Interrupt(timerBase);
    MAP_Timer_A_startCounter(timerBase, TIMER_A_UP_MODE);

    //Set Timer Flag
    switch(timerBase){
    case TIMER_A0_BASE:
        DAD_timerHasExpired0 = false;
        break;
    case TIMER_A1_BASE:
        DAD_timerHasExpired1 = false;
        break;
    case TIMER_A2_BASE:
        DAD_timerHasExpired2 = false;
        break;
    case TIMER_A3_BASE:
        DAD_timerHasExpired3 = false;
    }
}

bool DAD_Timer_Has_Finished(uint32_t timerBase){
    //Return Timer Flag
    switch(timerBase){
    case TIMER_A0_BASE:
        return DAD_timerHasExpired0;
    case TIMER_A1_BASE:
        return DAD_timerHasExpired1;
    case TIMER_A2_BASE:
        return DAD_timerHasExpired2;
    }
    return DAD_timerHasExpired3;
}

// Stop Timer
double DAD_Timer_Stop(uint32_t timerBase, Timer_A_UpModeConfig *timerConfig){
    // Stop timer
    MAP_Timer_A_stopTimer(timerBase);

    // Set Timer Flag
    switch(timerBase){
        case TIMER_A0_BASE:
            DAD_timerHasExpired0 = true;
            break;
        case TIMER_A1_BASE:
            DAD_timerHasExpired1 = true;
            break;
        case TIMER_A2_BASE:
            DAD_timerHasExpired2 = true;
            break;
        case TIMER_A3_BASE:
            DAD_timerHasExpired3 = true;
    }

    // Return current time
    return DAD_Timer_Get_Time(timerBase, timerConfig);
}

double DAD_Timer_Get_Time(uint32_t timerBase, Timer_A_UpModeConfig *timerConfig){
    // Decide if
    if(timerConfig->clockSource == TIMER_A_CLOCKSOURCE_ACLK)
        return MAP_Timer_A_getCounterValue(timerBase) * 1.024;  // Time in ms
    else if (timerConfig->clockSource == TIMER_A_CLOCKSOURCE_SMCLK) {
        float freq = MAP_CS_getSMCLK()/1000000.0;
        return MAP_Timer_A_getCounterValue(timerBase) / freq;   // Time in us
    }

    return 0;                                                   // Default :/
}

static void DAD_Timer_Set_Interrupt(uint32_t timerBase){
    // Decide which interrupt
    uint32_t interruptNum;
    switch(timerBase){
    case TIMER_A0_BASE:
        interruptNum = INT_TA0_0;
        DAD_timerHasExpired0 = false;
        break;
    case TIMER_A1_BASE:
        interruptNum = INT_TA1_0;
        DAD_timerHasExpired1 = false;
        break;
    case TIMER_A2_BASE:
        interruptNum = INT_TA2_0;
        DAD_timerHasExpired2 = false;
        break;
    case TIMER_A3_BASE:
        interruptNum = INT_TA3_0;
        DAD_timerHasExpired3 = false;
    }

    MAP_Interrupt_enableInterrupt(interruptNum);    // Enable timer interrupt
    MAP_Interrupt_enableMaster();                   // Enable interrupts
    MAP_Interrupt_disableSleepOnIsrExit();          // Don't sleep, boi
}

// Interrupt handlers
void TA0_0_IRQHandler(void)
{
    MAP_Timer_A_stopTimer(TIMER_A0_BASE);                                           // Stop timer

    // Clear interrupts
    MAP_Timer_A_clearInterruptFlag(TIMER_A0_BASE);                                  // Clear general timer interrupt flag
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,                         // Clear capture interrupt flag
                TIMER_A_CAPTURECOMPARE_REGISTER_0);

    // Debug - toggle LED to indicate interrupt not cleared
    if(MAP_Timer_A_getInterruptStatus(TIMER_A0_BASE) == TIMER_A_INTERRUPT_PENDING)  // Tests to see that interrupt was cleared
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);                    // If interrupt not cleared, turn on light

    // Set Timer Flag
    DAD_timerHasExpired0 = true;
}

void TA1_0_IRQHandler(void)
{
    MAP_Timer_A_stopTimer(TIMER_A1_BASE);                                           // Stop timer

    // Clear interrupt
    MAP_Timer_A_clearInterruptFlag(TIMER_A1_BASE);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,                         // Clear capture interrupt flag
                        TIMER_A_CAPTURECOMPARE_REGISTER_0);

    // Debug - toggle LED to indicate interrupt not cleared
    if(MAP_Timer_A_getInterruptStatus(TIMER_A1_BASE) == TIMER_A_INTERRUPT_PENDING)  // Tests to see that interrupt was cleared
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);                    // If interrupt not cleared, turn on light

    // Set Timer Flag
    DAD_timerHasExpired1 = true;
}


void TA2_0_IRQHandler(void)
{
    MAP_Timer_A_stopTimer(TIMER_A2_BASE);                                           // Stop timer

    // Clear interrupt
    MAP_Timer_A_clearInterruptFlag(TIMER_A2_BASE);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,                         // Clear capture interrupt flag
                        TIMER_A_CAPTURECOMPARE_REGISTER_0);

    // Debug - toggle LED to indicate interrupt not cleared
    if(MAP_Timer_A_getInterruptStatus(TIMER_A2_BASE) == TIMER_A_INTERRUPT_PENDING)  // Tests to see that interrupt was cleared
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);                    // If interrupt not cleared, turn on light

    // Set Timer Flag
    DAD_timerHasExpired2 = true;
}

#ifndef SET_TIMER_3_AS_SW_TIMER
void TA3_0_IRQHandler(void)
{
    MAP_Timer_A_stopTimer(TIMER_A3_BASE);                                           // Stop timer

    // Clear interrupt
    MAP_Timer_A_clearInterruptFlag(TIMER_A3_BASE);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,                         // Clear capture interrupt flag
                    TIMER_A_CAPTURECOMPARE_REGISTER_0);

    // Debug - toggle LED to indicate interrupt not cleared
    if(MAP_Timer_A_getInterruptStatus(TIMER_A3_BASE) == TIMER_A_INTERRUPT_PENDING)  // Tests to see that interrupt was cleared
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);                        // If interrupt not cleared, turn on light

    // Set Timer Flag
    DAD_timerHasExpired3 = true;
}
#endif
