#include <avr/io.h>

#ifndef GPIO_H
#define GPIO_H

/* Input/Output Registers */
#define DDR_PB0 DDRB
#define DDR_PB1 DDRB
#define DDR_PB2 DDRB
#define DDR_PB3 DDRB
#define DDR_PB4 DDRB
#define DDR_PB5 DDRB
#define DDR_PB6 DDRB
#define DDR_PB7 DDRB

#define DDR_PC0 DDRC
#define DDR_PC1 DDRC
#define DDR_PC2 DDRC
#define DDR_PC3 DDRC
#define DDR_PC4 DDRC
#define DDR_PC5 DDRC
#define DDR_PC6 DDRC
#define DDR_PC7 DDRC

#define DDR_PD0 DDRD
#define DDR_PD1 DDRD
#define DDR_PD2 DDRD
#define DDR_PD3 DDRD
#define DDR_PD4 DDRD
#define DDR_PD5 DDRD
#define DDR_PD6 DDRD
#define DDR_PD7 DDRD

/* Output value or Input Pull-Up Registers */
#define PORT_PB0 PORTB
#define PORT_PB1 PORTB
#define PORT_PB2 PORTB
#define PORT_PB3 PORTB
#define PORT_PB4 PORTB
#define PORT_PB5 PORTB
#define PORT_PB6 PORTB
#define PORT_PB7 PORTB

#define PORT_PC0 PORTC
#define PORT_PC1 PORTC
#define PORT_PC2 PORTC
#define PORT_PC3 PORTC
#define PORT_PC4 PORTC
#define PORT_PC5 PORTC
#define PORT_PC6 PORTC
#define PORT_PC7 PORTC

#define PORT_PD0 PORTD
#define PORT_PD1 PORTD
#define PORT_PD2 PORTD
#define PORT_PD3 PORTD
#define PORT_PD4 PORTD
#define PORT_PD5 PORTD
#define PORT_PD6 PORTD
#define PORT_PD7 PORTD

/* FUNCTIONS */
#define setAsOutput(px) { DDR_ ## px |= (1 << px); }
#define setAsInput(px)  { DDR_ ## px &= ~(1 << px); }
#define setHigh(px)     { PORT_ ## px |= (1 << px); }
#define setLow(px)      { PORT_ ## px &= ~(1 << px); }
#define enablePullUp(px) setHigh(px)
#define disablePullUp(px) setLow(px)

#endif