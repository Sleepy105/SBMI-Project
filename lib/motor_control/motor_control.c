/*
 *  motor_control.c
 *
 *  Created on: Oct 29, 2018
 *      Authors: Luís Sousa (lm.sousa@fe.up.pt), Leonor Santos (up201504515@fe.up.pt)
 *  
 *  Description:
 *      The code in this file is intended to provide a easy to use set of functions
 *      with the purpose of DC motor control with encoders.
 *      
 *      We use the PWM capabilities of the Atmega 328p Timer 0 to drive DC motors.
 */

#include <motor_control.h>

void initMotors () {

    for (int i = 0; i < HISTORY_SIZE; i++) {
        velocity[i] = 0;
    }
    _M1_encoder_counter = 0;
    _M2_encoder_counter = 0;

    TCCR0B = 0; // Stop the Timer
    TCCR0A = (1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00); // Set the Timer to Non-Inverting Fast-PWM Mode.
    TIMSK0 = 0; // Disable all interrupt calls
    _initAuxPins();
    setSpeed(0, 0);  // Don't start the motor running

    /* Finish the Fast-PWM configuration and start the Timer with a 1024 prescaler */
    TCCR0B = (0<<FOC0A) | (0<<FOC0B) | (1<<WGM02) | (1<<CS02) | (0<<CS01) | (1<<CS00);
}

void _initAuxPins () {

    /* Motor 1 */
    #ifdef M1_DIRECTION_PIN
    DDRD |= (1<<M1_DIRECTION_PIN);  // Set as output
    PORTD &= ~(1<<M1_DIRECTION_PIN);
    #endif

    #ifdef M1_BRAKE_PIN
        DDRD |= (1<<M1_BRAKE_PIN);  // Set as output
    #endif


    /* Motor 2 */
    #ifdef M2_DIRECTION_PIN
    DDRD |= (1<<M2_DIRECTION_PIN);  // Set as output
    PORTD &= ~(1<<M2_DIRECTION_PIN);
    #endif

    #ifdef M2_BRAKE_PIN
        DDRD |= (1<<M2_BRAKE_PIN);  // Set as output
    #endif
}


void setSpeed (int speed1, int speed2) {
    
    #ifdef M1_DIRECTION_PIN
    OCR0X(M1_DRIVE_PIN, 0);

    if ((speed1 < 0 && M1_INVERTED) || (speed1 >= 0 && !M1_INVERTED)) {
        PORTD &= ~(1<<M1_DIRECTION_PIN);
    }
    else {
        PORTD |= (1<<M1_DIRECTION_PIN);
    }
    #endif

    #ifdef M2_DIRECTION_PIN
    OCR0X(M2_DRIVE_PIN, 0);

    if ((speed2 < 0 && M1_INVERTED) || (speed2 >= 0 && !M1_INVERTED)) {
        PORTD &= ~(1<<M2_DIRECTION_PIN);
    }
    else {
        PORTD |= (1<<M2_DIRECTION_PIN);
    }
    #endif

    if (speed1 > 0) {
        speed1 = -speed1;
    }
    if (speed2 > 0) {
        speed2 = -speed2;
    }

    OCR0X(M1_DRIVE_PIN, (uint8_t)(speed1*255/100));
    OCR0X(M2_DRIVE_PIN, (uint8_t)(speed2*255/100));

    #ifdef M1_BRAKE_PIN
    if (speed1 == 0)
        PORTD |= (1<<M1_BRAKE_PIN);
    else
        PORTD &= ~(1<<M1_BRAKE_PIN);
    #endif

    #ifdef M2_BRAKE_PIN
    if (speed2 == 0)
        PORTD |= (1<<M2_BRAKE_PIN);
    else
        PORTD &= ~(1<<M2_BRAKE_PIN);
    #endif
}

void updateOdometry (uint16_t millis) {
    for (int i = HISTORY_SIZE-1; i > 0; i--) {
        velocity[i] = velocity[i-1];
    }

    cli(); // Prevent interrupt call from changing the counter's values
    
    velocity[0] = (float)(((_M1_encoder_counter+_M2_encoder_counter)/2)/millis); // Average the velocity of the 2 motors
    _M1_encoder_counter = 0;
    _M2_encoder_counter = 0;
    
    sei(); // Re-enable interrupt calls
}