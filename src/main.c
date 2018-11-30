/*
 *  main.c
 *
 *  Created on: Oct 29, 2018
 *      Author: Luís Sousa, Leonor Santos
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <motor_control.h>


/*******************************/
/*                             */
/*    Generic configuration    */
/*                             */
/*******************************/

//#define DEBUG
//#define DEMO

#define INITIAL_STATE 0
#define BREAKDOWN_ENTRY_STATE 0
#define ODOMETRY_TIME_RESOLUTION 1000

uint8_t state = 0;  // Current state of the state-machine
uint8_t nstate = 0; // Next state of the state-machine
uint16_t odometryTime = 0;

/**
 * @brief Setups all External Interruptions
 * 
 */
void initExternalInterrupts() {
    EICRA |= (1<<ISC11) | (1<<ISC10) | (1<<ISC01) | (1<<ISC00); // Both external Interrupt calls at RE
    EIMSK |= (1 << M1_ENCODER_INTERRUPT_PIN) | (1 << M2_ENCODER_INTERRUPT_PIN); // Enable the external interrupt calls
    EIFR |= (1<<INTF1) | (1<<INTF0);    // Clear Flag Register by writting '1's to the corresponding positions
}

ISR(INT0_vect) {    // M1_ENCODER_INTERRUPT_PIN Interrupt Call
    _M1_encoder_counter += 1;
}

ISR(INT1_vect) {    // M2_ENCODER_INTERRUPT_PIN Interrupt Call
    _M2_encoder_counter += 1;
}

/**
 * @brief Condenses all hardware configurations to one function call
 * 
 */
void initHardware() {
    cli(); // Disable all interrupts during hardware configuration

    initExternalInterrupts();
    initMotors();

    /* Activate Interrupts */
    sei();
}

int main() {
    /* Check the reason for the reset */
    switch(MCUCR^0b00001111) {
        case PORF:  // Power-On Reset Flag
            // Do nothing.
            break;
        case EXTRF: // External Reset Flag
            // Do nothing.
            break;
        case BORF:  // Brown-out Reset Flag
            // TODO: Save all important RAM data to EEPROM QUICK!!!
            break;
        case WDRF:  // Watchdog System Reset Flag
            // TODO: Either go into Breakdown mode or log the reset to the EEPROM
            break;
        default:
            state = BREAKDOWN_ENTRY_STATE;
            break;
    }
    MCUCR = 0; // Clear the Reset Flag Register

    initHardware();
    odometryTime = ODOMETRY_TIME_RESOLUTION; // Initialize counter

    while(1) {
        if (!odometryTime) {
            updateOdometry(ODOMETRY_TIME_RESOLUTION);   // Update Odometry values
            odometryTime = ODOMETRY_TIME_RESOLUTION;    // Reset the time variable
        }
        
        switch(state) {
            case 0:
                setSpeed(100,100);
                break;
            case 1:
                break;
            default:
                nstate = BREAKDOWN_ENTRY_STATE;
                break;
        }

        nstate = state; // Update 'state' variable
    }
    return 0;
}