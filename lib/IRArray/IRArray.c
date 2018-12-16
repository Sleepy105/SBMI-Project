#include <IRArray.h>

void initArrayHardware() {
    DDRB &= ~(1<<IR_0 | 1<<IR_1 | 1<<IR_2 | 1<<IR_3 | 1<<IR_4); // IR sensor pins as inputs
    PORTB |= (1<<IR_0 | 1<<IR_1 | 1<<IR_2 | 1<<IR_3 | 1<<IR_4); // pull-up resistor on
}

uint8_t updateSensorValuesArray() {
    return PINB & 0x1F;
}

