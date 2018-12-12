/*
 *  BTcomms.h
 *
 *  Created on: Nov 30, 2018
 *      Author: Luís Sousa, Leonor Santos
 * 
 *  Library to handle a 5 IR sensor array.
 *  The sensors used output a simple digital signal, given the reflection of IR radiation from the surface in question.
 *  A potentiometer is also present in these sensors for adjustment of the trigger distance. 
 */

#include <avr/io.h>

#ifndef __IR_ARRAY__
#define __IR_ARRAY__

#define ARRAY_SIZE 5

#define IR_0 PC0        /*The 5 IR sensors are displayed as an array. Each one has its order number*/
#define IR_1 PC1
#define IR_2 PC2
#define IR_3 PC3
#define IR_4 PC4

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