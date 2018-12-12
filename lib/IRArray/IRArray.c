#include <IRArray.h>

void initArrayHardware() {
    DDRC &= ~( (1<<IR_0) | (1<<IR_1) | (1<<IR_2) | (1<<IR_3) | (1<<IR_4) );
    PORTC |= (1<<IR_0) | (1<<IR_1) | (1<<IR_2) | (1<<IR_3) | (1<<IR_4);
}

uint8_t updateSensorValuesArray() {
    return (PINC & 0b00011111);
}