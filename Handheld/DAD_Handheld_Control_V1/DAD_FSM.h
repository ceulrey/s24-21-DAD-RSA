/*
 * DAD_FSM.h
 *
 *  Created on: Mar 8, 2023
 *      Author: Max Engel
 *
 *      State machine for controlling handheld device.
 */

#ifndef DAD_FSM_H_
#define DAD_FSM_H_

// Interface includes
#include <HAL/DAD_UART.h>
#include <DAD_Utils/DAD_Interface_Handler.h>
#include <DAD_Packet_Handler.h>

#ifdef PRIORITIZE_FFT
#define MIN_PACKETS_TO_PROCESS 200
#else
#define MIN_PACKETS_TO_PROCESS 25
#endif

#define RSA_RX_TIMEOUT_PERIOD_MS 10000

typedef enum {STARTUP, RSA_READ, HANDLE_PERIPH, HANDLE_RSA_TIMEOUT, STOP_STATE} FSMstate;   // TODO add states as necessary

// Device control
void DAD_FSM_control(FSMstate *state, DAD_Interface_Struct* interfaceStruct);

#endif /* DAD_FSM_H_ */
