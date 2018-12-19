#include <IRArray.h>

void initArrayHardware() {
    DDRC &= ~IR_ARRAY_MASK; // IR sensor pins as inputs
    PORTC |= IR_ARRAY_MASK; // pull-up resistor on
}

uint8_t updateSensorValuesArray() {
    return PINC & IR_ARRAY_MASK;
}

