/*
 *  motor_control.h
 *
 *  Created on: Oct 29, 2018
 *      Author: Luís Sousa, Leonor Santos
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#ifndef __MOTOR_CONTROL__
#define __MOTOR_CONTROL__

#define TRUE 1
#define FALSE 0

/*******************************/
/*                             */
/*    Motor 1 configuration    */
/*                             */
/*******************************/
#define M1_DRIVE_PIN PD6
#define M1_ENCODER_INTERRUPT_PIN INT0   // PD2
#define M1_DIRECTION_PIN PD4
//#define M1_BRAKE_PIN 0
#define M1_INVERTED TRUE


/*******************************/
/*                             */
/*    Motor 2 configuration    */
/*                             */
/*******************************/
#define M2_DRIVE_PIN PD5
#define M2_ENCODER_INTERRUPT_PIN INT1   // PD3
#define M2_DIRECTION_PIN PD7
//#define M2_BRAKE_PIN 0
#define M2_INVERTED FALSE


/*******************************/
/*                             */
/*    Generic configuration    */
/*                             */
/*******************************/
#define HISTORY_SIZE 10
#define WHEEL_RADIUS 0.0065
#define CPR 11


/*****************************************/
/*                                       */
/*         END OF CONFIGURATIONS         */
/*                                       */
/*****************************************/

/**
 * @brief Distance Per Counter: Calculates the distance travelled between encoder pulses
 * 
 */
#define DPC ((2 * M_PI * (WHEEL_RADIUS*WHEEL_RADIUS))/CPR)

/**
 * @brief Initializes Timer0 with a 1024 prescaler, in Fast-PWM mode to use
 * 2 motors. Also initializes auxilary variables and pins (see _initAuxPins()).
 * 
 */
void initMotors ();

/**
 * @brief Initializes all defined auxiliary motor control pins.
 * 
 */
void _initAuxPins ();

/**
 * @brief Set the Speed of the 2 motors by changing the OCR0x registers
 * Acceptable values range from -100 to 100.
 * 
 * @param speed1 Motor1 speed from -100 to 100
 * @param speed2 Motor2 speed from -100 to 100
 */
void setSpeed (int speed1, int speed2);

/**
 * @brief Array to hold the robot's velocity over time
 * 
 */
volatile float velocity[HISTORY_SIZE];

/**
 * @brief Counter variable for Motor 1 odometry
 * Updated every time the encoder of that motor triggers a interrupt call.
 * 
 */
volatile uint8_t _M1_encoder_counter;

/**
 * @brief Counter variable for Motor 1 odometry
 * Updated every time the encoder of that motor triggers a interrupt call.
 * 
 */
volatile uint8_t _M2_encoder_counter;

/**
 * @brief Update the values of the velocity array.
 * Shifts all positions to the right and adds the new value at position 0.
 * Velocity value is calculated by averaging the 2 motors's movement over millis time.
 * To maintain high accuracy of mesurements, "millis" should be constant
 * 
 * @param millis Time resolution of the speed array
 */
void updateOdometry (uint16_t millis);

#endif // __MOTOR_CONTROL__