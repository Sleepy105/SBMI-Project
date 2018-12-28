/**
 * @file IRArray.h
 * @author Luís Sousa, Leonor Santos
 * @date 30 Nov 2018
 * @brief Library to handle a 5 IR sensor array.
 * 
 * The sensors used output a simple digital signal, given the reflection of IR radiation from the surface in question.
 * A potentiometer is also present in these sensors for adjustment of the trigger distance.
 */

#include <avr/io.h>

#ifndef __IR_ARRAY__
#define __IR_ARRAY__

#define IR_0 PC5
#define IR_1 PC1
#define IR_2 PC2
#define IR_3 PC3
#define IR_4 PC4

#define IR_ARRAY_MASK (_BV(IR_0) | _BV(IR_1) | _BV(IR_2) | _BV(IR_3) | _BV(IR_4))

/**
 * @brief Initializes the hardware required for the proper function of the sensor array.
 * 
 */
void initArrayHardware();

/**
 * @brief Fetches the current values of the sensors
 * 
 * Returns a byte, of which every bit represents the status of a sensor in the array.
 * The byte is populated from the LSB to the MSB.
 * The sensor status bits are arranged in the same order as the physical sensor array.
 * 
 * @return uint8_t Status bits
 */
uint8_t updateSensorValuesArray();

#endif