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

#define LEFTMOST_SENSOR_PIN IR_0
#define LEFT_CENTRE_SENSOR_PIN IR_1
#define CENTER_SENSOR_PIN IR_2
#define RIGHT_CENTRE_SENSOR_PIN IR_3
#define RIGHTMOST_SENSOR_PIN IR_4

#define START_BUTTON PC0

#define BASE_SPEED  15
#define DECREMENT   3

#define INITIAL_STATE 0
#define BREAKDOWN_ENTRY_STATE 255
#define ODOMETRY_TIME_RESOLUTION 1000

uint8_t state = 0;                  // Current state of the state-machine
uint8_t nstate = 0;                 // Next state of the state-machine
volatile uint8_t istate = 0;        // Variable to prevent race-conditions to the state-machine's 'state'.
volatile uint8_t i_flag = FALSE;    // Flag to tell the main-loop if a request is made by a interrupt to change the current state-machine's state
volatile uint16_t odometryTime = 0;
uint8_t mcucr_copy;
volatile char lastReceivedChar = 0;
volatile uint8_t newCharFlag = FALSE;
volatile uint16_t time1 = 0;
volatile uint8_t IR_vector = 0;
uint8_t prev_IR_vector = 0;
uint8_t IR_vector_changed = FALSE;
uint8_t laps = 0;                   // Laps completed around the track

/**
 * @brief If a Inturrupt is called and has no ISR associated, this function is called, instead of a system reset occuring
 * 
 */
ISR(BADISR_vect) {
    istate = BREAKDOWN_ENTRY_STATE;
    i_flag = TRUE;
}

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
    // TODO: Save important variables to EEPROM
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
 * Supposedly, it is triggered every 10ms.
 * 
 */
ISR(TIMER1_COMPA_vect) {
    if (odometryTime) {
        odometryTime -= 10;
    }
    if (time1) {
        time1 -= 10;
    }
    // FIXME: non multiples of 10 result in shit!
}

/**
 * @brief Condenses all hardware configurations to one function call
 * 
 */
void initHardware() {
    cli(); // Disable all interrupts during hardware configuration

    initExternalInterrupts();
    initMotors();
    initWatchdogTimer();
    setReceiveCallback(&handleReceivedByte);
    initBTComms();
    init_TC1_10ms();
    initArrayHardware();
    DDRC &= ~(1<<START_BUTTON);  // Set as Input // FIXME: Change this
    PORTC |= (1<<START_BUTTON); // Enable Pull-up resistor
    DDRB |= ~(1<<PB5);
    PORTB &= ~(1<<PB5);

    /* Activate Interrupts */
    sei();
}


/**
 * @brief Disables unused peripherals to save power.
 * 
 */
void disableUnusedHardware() {
    // TODO: Disable unused peripherals to save power.
}

int main() {
    mcucr_copy = MCUCR^0b00001111;
    cli();
    disableWatchdogTimer();

    /* Check the reason for the reset */
    switch(mcucr_copy) {
        case PORF:  // Power-On Reset Flag
            // TODO: Check if the siganture is present. If not, prime the EEPROM
            break;
        case EXTRF: // External Reset Flag
            // TODO: Same as PORF???? IDK... yet.
            break;
        case BORF:  // Brown-out Reset Flag
            // TODO: Print reason out to LCD
            // TODO: Interrupt is not enabled
            break;
        case WDRF:  // Watchdog System Reset Flag
            // TODO: Either go into Breakdown mode or log the reset to the EEPROM
            // If needed, read saved data from EEPROM
            break;
        default:
            state = BREAKDOWN_ENTRY_STATE;
            break;
    }
    MCUCR = 0; // Clear the Reset Flag Register

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
        IR_vector_changed = !(IR_vector == prev_IR_vector);

        if (i_flag) {
            state = istate;
            i_flag = FALSE;
        }

        if (!odometryTime) {
            updateOdometry(ODOMETRY_TIME_RESOLUTION);   // Update Odometry values
            odometryTime = ODOMETRY_TIME_RESOLUTION;    // Reset the time variable
        }

        if (PINC & (1<<START_BUTTON)) {
            state = 0;
        }
        
        /* State Machine */
        switch(state) {
            case 0:
                PORTB |= (1<<PB5);
                setSpeed(0,0);
                if (!(PINC & (1<<START_BUTTON))) {
                    nstate = 1;
                    time1 = 3000;
                }
                break;
            case 1:
                PORTB &= ~(1<<PB5);
                if (!time1) {
                    nstate = 2;
                    setSpeed(BASE_SPEED,BASE_SPEED);
                }
                break;
            case 2:
                PORTB |= (1<<PB5);
                if (!IR_vector) {   // All sensors detecing a line
                    nstate = 254;
                    setSpeed(BASE_SPEED,BASE_SPEED);
                    laps++;
                }
                else if (IR_vector_changed && (IR_vector ^ 0x1F)) { // Sensor values have changed and there is at least one detecting the line
                        setSpeed(BASE_SPEED - (!(IR_vector & (1<<LEFTMOST_SENSOR_PIN)) * DECREMENT) - (!(IR_vector & (1<<LEFT_CENTRE_SENSOR_PIN)) * DECREMENT), BASE_SPEED - (!(IR_vector & (1<<RIGHT_CENTRE_SENSOR_PIN)) * DECREMENT) - (!(IR_vector & (1<<RIGHTMOST_SENSOR_PIN)) * DECREMENT));
                }
                break;
            case 254:
                if (IR_vector) {
                    nstate = 2;
                }
                break;
            case 255: // BREAKDOWN_ENTRY_STATE
                setSpeed(0,0);          // Stop all movement
                cli();                  // Disable all interrupts
                disableWatchdogTimer(); // Disable the watchdog timer
                while(1);               // Wait for reset
                break;
            default:
                nstate = BREAKDOWN_ENTRY_STATE;
                break;
        }

        state = nstate; // Update 'state' variable
    }
    return 0;
}