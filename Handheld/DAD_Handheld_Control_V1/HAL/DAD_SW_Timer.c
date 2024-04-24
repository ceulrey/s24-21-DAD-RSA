/*
 * DAD_SW_Timer.c
 *
 *  Created on: Apr 3, 2023
 *      Author: Max
 */

#include <HAL/DAD_Timer.h>
#include <HAL/DAD_SW_Timer.h>

static Timer_A_UpModeConfig swTimerConfig;
static volatile uint32_t tickCounter;

#ifdef SET_TIMER_3_AS_SW_TIMER
// Initializes the hardware timer.
void DAD_SW_Timer_initHardware(){
    DAD_Timer_Initialize_ms(DAD_SW_TIMER_RESOLUTION_MS, DAD_SW_TIMER_HANDLE, &swTimerConfig);
    DAD_Timer_Start(DAD_SW_TIMER_HANDLE);
}

// Get ms since starting (max of 32 bits)
bool DAD_SW_Timer_getMS(uint64_t* timeToReturn){
    // Overflow detection
    bool returnVal = true;
    if(tickCounter * DAD_SW_TIMER_RESOLUTION_MS < *timeToReturn)
        returnVal = false;

    *timeToReturn = (tickCounter * DAD_SW_TIMER_RESOLUTION_MS);
    return returnVal;
}

void TA3_0_IRQHandler(void)
{
    tickCounter++;  // Increment ticks
    uint32_t intStatus = Timer_A_getEnabledInterruptStatus(EUSCI_A3_BASE);

    // Clear interrupt
    MAP_Timer_A_clearInterruptFlag(TIMER_A3_BASE);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,                         // Clear capture interrupt flag
                    TIMER_A_CAPTURECOMPARE_REGISTER_0);

    // Debug - toggle LED to indicate interrupt not cleared
    if(MAP_Timer_A_getInterruptStatus(TIMER_A3_BASE) == TIMER_A_INTERRUPT_PENDING)  // Tests to see that interrupt was cleared
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);                        // If interrupt not cleared, turn on light

}
#endif
