/*
 *  main.c
 *
 *  Created on: Oct 29, 2018
 *      Author: Luís Sousa, Leonor Santos
 */

#ifndef F_CPU
#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

#include <motor_control.h>
#include <BTcomms.h>
#include <IRArray.h>


/*******************************/
/*                             */
/*    Generic configuration    */
/*                             */
/*******************************/

//#define DEBUG
//#define DEMO

#define EEPROM_MIN 0
#define EEPROM_MAX 1023
#define SIG_16 0b0101110101110100   // Program EEPROM signature.
#define SIG_16_POS 0
#define DISTANCE_COUNTER_POS 2
#define WDT_RESET_COUNTER_POS 6

#define LEFTMOST_SENSOR_PIN IR_0
#define LEFT_CENTRE_SENSOR_PIN IR_1
#define CENTER_SENSOR_PIN IR_2
#define RIGHT_CENTRE_SENSOR_PIN IR_3
#define RIGHTMOST_SENSOR_PIN IR_4

#define START_BUTTON PC0
#define STATUS_LED PB5

#define BASE_SPEED  10
#define DECREMENT   10

#define INITIAL_STATE 0
#define ODOMETRY_TIME_RESOLUTION 1000

uint8_t state = 0;                  // Current state of the state-machine
uint8_t nstate = 0;                 // Next state of the state-machine
volatile uint16_t odometryTime = 0;
uint8_t mcucr_copy;
volatile char lastReceivedChar = 0;
volatile uint8_t newCharFlag = FALSE;
volatile uint16_t time1 = 0;
volatile uint8_t IR_vector = 0;
uint8_t prev_IR_vector = 0;
uint8_t IR_vector_changed = FALSE;
uint8_t laps = 0;                   // Laps completed around the track
float distance = 0;

/**
 * @brief Setups all External Interruptions
 * 
 */
void initExternalInterrupts() {
    EICRA |= (1<<ISC11) | (1<<ISC10) | (1<<ISC01) | (1<<ISC00); // Both external Interrupt calls at RE
    EIMSK |= (1 << M1_ENCODER_INTERRUPT_PIN) | (1 << M2_ENCODER_INTERRUPT_PIN); // Enable the external interrupt calls
    EIFR |= (1<<INTF1) | (1<<INTF0);    // Clear Flag Register by writting '1's to the corresponding positions
}

/**
 * @brief M1_ENCODER_INTERRUPT_PIN Interrupt Call
 * 
 */
ISR(INT0_vect) {
    _M1_encoder_counter += 1;
}

/**
 * @brief M2_ENCODER_INTERRUPT_PIN Interrupt Call
 * 
 */
ISR(INT1_vect) {
    _M2_encoder_counter += 1;
}

/**
 * @brief Saves important variables to EEPROM
 * 
 */
void emergencySaveToEEPROM() {
    eeprom_busy_wait();
    uint8_t wdt_counter_value = eeprom_read_byte((void*)WDT_RESET_COUNTER_POS);
    wdt_counter_value++;
    eeprom_busy_wait();
    eeprom_write_byte((void*)WDT_RESET_COUNTER_POS, wdt_counter_value);
    
    eeprom_busy_wait();
    eeprom_update_float((void*)DISTANCE_COUNTER_POS, distance);
}

/**
 * @brief Enables a 0.5 second Watchdog Timer that trigger both the Interrupt Call followed by a System Reset
 * 
 */
void initWatchdogTimer() {
    wdt_reset();                    // 'Pet the Dog'
    WDTCSR |= (1<<WDCE) | (1<<WDE); // Unlock prescaler changes for 4 clock cycles
    /* Set Timeout to 0.5 seconds, clear Interrupt flag and enable both Interrupt and system reset modes */
    WDTCSR = (1<<WDIF) | (1<<WDE) | (1<<WDIE) | (1<<WDP2) | (1<<WDP0);
}

/**
 * @brief Disables the Watchdog Timer. Both the Interrupt and System Reset Enable bits are also disabled.
 * 
 */
void disableWatchdogTimer() {
    wdt_reset();            // 'Pet the Dog'
    MCUSR &= ~(1<<WDRF);    // Datasheet Recomendation to prevent time-out loops
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional time-out */
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
}

/**
 * @brief Interrupt Service Routine for when the Watchdog Timer is triggered
 * 
 */
ISR(WDT_vect) {
    emergencySaveToEEPROM();
}

/**
 * @brief Callback function to handle the byte received by the UART interface
 * 
 * @param rxChar byte received by the UART interface
 */
void handleReceivedByte(char rxChar) {
    lastReceivedChar = rxChar;
    newCharFlag = TRUE;
}

/**
 * @brief Initializes Timer1 to trigger a interrupt every 10ms (TIMER1_COMPA_vect)
 * 
 */
void init_TC1_10ms() {
    TCCR1B = 0;                         // STOP the timer
    TIFR1 = (7<<TOV1) | (1<<ICF1);      // Clear flag register
    TCCR1A = 0;                         // Set mode...
    TCCR1B = (1<<WGM12);                // ...to CTC
    OCR1A = 625;                        // OCR1A
    TIMSK1 = (1<<OCIE1A);               // Enable interrupt on compare with OCR1A
    TCCR1B |= 4;                        // Set the TP to 256, therefore also starting the timer
}

/**
 * @brief Handles the countdown of all application timers.
 * 
 * It is triggered every 10ms.
 * 
 */
ISR(TIMER1_COMPA_vect) {
    if (odometryTime) {
        if (odometryTime < 10) {
            odometryTime = 10;
        }
        odometryTime -= 10;
    }
    if (time1) {
        if (time1 < 10) {
            time1 = 10;
        }
        time1 -= 10;
    }
}

void initEEPROM() {
    eeprom_busy_wait();
    uint16_t wd = eeprom_read_word((void*)SIG_16_POS);
    if (wd != SIG_16) {
        for (uint16_t i = EEPROM_MIN; i <= EEPROM_MAX; i++) {
            eeprom_busy_wait();
            eeprom_write_byte((void*)i, 0xFF);
        }
        eeprom_busy_wait();
        eeprom_write_word((void*)SIG_16_POS, SIG_16);
        eeprom_busy_wait();
        eeprom_write_float((void*)DISTANCE_COUNTER_POS, 0x00);
        eeprom_busy_wait();
        eeprom_write_byte((void*)WDT_RESET_COUNTER_POS, 0x00);
    }
}

/**
 * @brief Condenses all hardware configurations to one function call
 * 
 */
void initHardware() {
    cli(); // Disable all interrupts during hardware configuration

    //initExternalInterrupts();
    initMotors();
    initWatchdogTimer();
    setReceiveCallback(&handleReceivedByte);
    //initBTComms();
    init_TC1_10ms();
    initArrayHardware();
    DDRC &= ~(1<<START_BUTTON);  // Set as Input
    PORTC |= (1<<START_BUTTON); // Enable Pull-up resistor
    DDRB |= ~(1<<STATUS_LED);
    PORTB &= ~(1<<STATUS_LED);

    /* Activate Interrupts */
    sei();
}

void breakdown() {
    setSpeed(0,0);          // Stop all movement
    cli();                  // Disable all interrupts
    disableWatchdogTimer(); // Disable the watchdog timer
    while(1);               // Wait for reset
}

/**
 * @brief If a Inturrupt is called and has no ISR associated, this function is called, instead of a system reset occuring
 * 
 */
ISR(BADISR_vect) {
    breakdown();
}


/**
 * @brief Disables unused peripherals to save power.
 * 
 */
void disableUnusedHardware() {
    // TODO: Disable unused peripherals to save power.
}

int main() {
    mcucr_copy = MCUSR & 0b00001111;
    cli();
    disableWatchdogTimer();

    /* Check the reason for the reset */
    switch(mcucr_copy) {
        case (1<<PORF):  // Power-On Reset Flag
        case (1<<EXTRF): // External Reset Flag
            initEEPROM();
            eeprom_busy_wait();
            distance = eeprom_read_float((void*)DISTANCE_COUNTER_POS);
            break;
        case (1<<WDRF):  // Watchdog System Reset Flag
        case (1<<BORF):  // Brown-out Reset Flag
        default:
            //breakdown();
            break;
    }
    MCUSR = 0; // Clear the Reset Flag Register

    /* Hardware and Timer Initializations */
    initHardware();
    disableUnusedHardware();
    odometryTime = ODOMETRY_TIME_RESOLUTION; // Initialize counter

    /* Main Program Loop */
    while(1) {
        wdt_reset();    // 'Pet the Dog'

        /* Update IR vector variables according to the sensors */
        prev_IR_vector = IR_vector;
        IR_vector = updateSensorValuesArray();
        IR_vector_changed = (IR_vector != prev_IR_vector);

        if (!odometryTime) {
            updateOdometry(ODOMETRY_TIME_RESOLUTION);   // Update Odometry values
            odometryTime = ODOMETRY_TIME_RESOLUTION;    // Reset the time variable

            distance += velocity[0] * ODOMETRY_TIME_RESOLUTION/1000;
        }

        if (PINC & (1<<START_BUTTON)) {
            state = 0;
        }

        // TODO: Change to add all commands of BT comms in function and separate state
        if (newCharFlag) {
            if ('F' == lastReceivedChar) {
                //PORTB |= (1<<STATUS_LED);
            }
            else {
                //PORTB &= ~(1<<STATUS_LED);
            }
            newCharFlag = FALSE;
        }
        
        /* State Machine */
        switch(state) {
            case 0:
                //PORTB |= (1<<STATUS_LED);
                setSpeed(0,0);
                if (!(PINC & (1<<START_BUTTON))) {
                    nstate = 1;
                    time1 = 3000;
                }
                break;
            case 1:
                //PORTB &= ~(1<<STATUS_LED);
                if (!time1) {
                    nstate = 2;
                    setSpeed(BASE_SPEED,BASE_SPEED);
                }
                break;
            case 2:
                //PORTB |= (1<<STATUS_LED);
                if (0 == IR_vector) {   // All sensors detecing a line
                    nstate = 254;
                    setSpeed(BASE_SPEED,BASE_SPEED);
                    laps++;
                }
                else if (IR_vector_changed){// && (IR_vector ^ 0x3E)) { // Sensor values have changed and there is at least one detecting the line
                    int l = BASE_SPEED,
                        r = BASE_SPEED;
                    if ( 0==(IR_vector & (1<<LEFTMOST_SENSOR_PIN)) ) {
                        l = 0;
                        PORTB |= (1<<STATUS_LED);
                    }
                    else {
                        PORTB &= ~(1<<STATUS_LED);
                    }
                    if ( 0==(IR_vector & (1<<RIGHTMOST_SENSOR_PIN)) ) {
                        r = 0;
                    }
                    setSpeed(l,r);
                }
                break;
            case 254:
                if (0 != IR_vector) {
                    nstate = 2;
                }
                break;
            default:
                breakdown();
                break;
        }

        state = nstate; // Update 'state' variable
    }
    return 0;
}