/*
 *  BTcomms.c
 *
 *  Created on: Nov 30, 2018
 *      Author: Luís Sousa, Leonor Santos
 * 
 *  Library to communicate with a HC-05 Bluetooth Module.
 */

#include <BTcomms.h>

void initBTComms() {
    UCSR0B &= ~((1<<RXEN0) | (1<<TXEN0));   // Disable all communication during set-up


    /* Define the Baud Rate and if Double Speed Mode is needed */
    #include <util/setbaud.h>
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    #if USE_2X
        UCSR0A |= (1<<U2X0);
    #else
        UCSR0A &= ~(1<<U2X0);
    #endif

    UCSR0A &= ~((1<<FE0)    // Clear the Frame Error Flag (Set to 0)
        | (1<<DOR0)         // Clear the Data OverRun Flag (Set to 0)
        | (1<<UPE0)         // Clear the Parity Error Flag (Set to 0)
        | (1<<MPCM0));      // Disable Multi-processor Mode (Set to 0)
    UCSR0A |= (1<<TXC0);    // Clear the Transmit Complete Flag

    UCSR0B |= (1<<RXCIE0);  // Enable RX Complete Interrupt (USART_RX_vect)
    UCSR0B &= ~((1<<TXCIE0) // Disable TX Complete Interrupt
        | (1<<UDRIE0));


    /* Set up the data packet size. */
    /* 9600N81 */
    UCSR0B &= ~(1<<UCSZ02); // 8 data...
    UCSR0C = (3<<UCSZ00)    // ...bits
        | (0<<UPM01)        // No parity...
        | (0<<UPM00)        // ...bit
        | (0<<USBS0)        // 1 stop bit
        | (0<<UMSEL01)      // USART Mode:
        | (0<<UMSEL00)      // ...Asynchronous USART
        | (0<<UCPOL0);      // Bit used only for symchronous mode. Required to be '0' in async mode.

    UCSR0B |= (1<<RXEN0);   // Enable RX communication
}

void setReceiveCallback(void* callbackFunctionPointer) {
    __callback_function = callbackFunctionPointer;
}

ISR(USART_RX_vect) {
    (*__callback_function)(UDR0);
}