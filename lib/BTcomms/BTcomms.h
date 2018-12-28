/**
 * @file BTcomms.c
 * @author Luís Sousa, Leonor Santos
 * @date 30 Nov 2018
 * @brief Library to communicate with a HC-05 Bluetooth Module.
 *
 * This library only supports the receival of information from the BT Module.
 * Although transmission of data is possible, it is not facilitated by the functions defined here.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef __BT_COMMS__
#define __BT_COMMS__

#define BAUD_TOL 2
#define BAUD 9600

/**
 * @brief Initializes and configures the UART interface to be able to communicate with a HC-05 Bluetooth Module.
 * 
 * This module expects and transmits frames in a 9600N81 setting.
 * It also initializes a interrupt so that when data is received, a jump to the USART_RX_vect vector.
 * A ISR is needed to handle that interruption.
 * CAUTION: "the receive complete routine must read the received data
 * from UDR in order to clear the RXC Flag, otherwise a new interrupt will occur once the interrupt routine
 * terminates" causing a interrupt loop!
 */
void initBTComms();

/**
 * @brief Pointer to the callback function used to handle a received byte.
 * 
 */
void (*__callback_function)(char);

/**
 * @brief Set the Receive Callback pointer to the user-defined function
 * 
 * @param callbackFunctionPointer
 */
void setReceiveCallback(void* callbackFunctionPointer);

#endif