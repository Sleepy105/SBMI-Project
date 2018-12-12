#include <IRArray.h>

void initArrayHardware() {
<<<<<<< HEAD
    DDRC~&= ~(1<<IR_0 | 1<<IR_1 | 1<<IR_2 | 1<<IR_3 | 1<<IR_4);     /* IR sensor pins as inputs*/
    PORTC |= (1<<IR_0 | 1<<IR_1 | 1<<IR_2 | 1<<IR_3 | 1<<IR_4);         /* pull-up resistor on*/
}

uint8_t updateSensorValuesArray() {
    
    IR_vector=  PINC &  0<1F;

    return IR_vector;

}

