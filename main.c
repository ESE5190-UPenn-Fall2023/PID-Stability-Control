// Constants for Clock and Baud Rate
#define F_CPU 16000000UL
#define BAUD 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD * 16UL))) - 1)

// Pin Definitions
#define TRIG_PIN PD3
#define ECHO_PIN PB0
#define SERVO_PWM_PIN PD5

// ADC Channels for KP, KI, KD
#define KP_ADC_CHANNEL 0  // PC0
#define KI_ADC_CHANNEL 1  // PC1
#define KD_ADC_CHANNEL 2  // PC2

// Measurement and Servo Parameters
#define MAX_TRACK_LENGTH 32
#define TARGET_POSITION (MAX_TRACK_LENGTH / 2)
#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define MIN_ANGLE_SETTABLE 10
#define MAX_ANGLE_SETTABLE 140
#define LEVEL_ANGLE (MIN_ANGLE + MAX_ANGLE) / 2
#define SERVO_PWM_FREQ 50
#define SERVO_MIN 900
#define SERVO_MAX 4900

// PID Controller Parameters
#define MAX_KP_VALUE 10.0
#define MAX_KI_VALUE 5.0
#define MAX_KD_VALUE 5.0
#define I_MAX 70  // max integral value
#define I_MIN 5  // min integral value

// Debug flag
#define DEBUG 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>
#include "uart.h"

// Ultrasonic Measurement Modes
typedef enum {
    TRIG_MODE,
    REC_MODE,
    SEND_MODE
} SystemMode;

// Global Variables
volatile SystemMode system_mode = TRIG_MODE;
volatile long tick_end = 0;
volatile long tick_start = 0;
float previous_angle = MIN_ANGLE, previous_error = 0, integral = 0;

// Function Prototypes
void clk_init(void);

void ADC_init(void);

float read_ADC(int channel, float max_value);

void init_HC_SR04(void);

void setup_servo_PWM(void);

void set_servo_angle(uint8_t angle);

float calculate_PID(float distance);

void handle_timer1_capture(void);

// Main function
int main(void) {
    UART_init(BAUD_PRESCALER);
    clk_init();
    init_HC_SR04();
    setup_servo_PWM();
    ADC_init();
    sei();

    float distance, angle;

    while (1) {
        switch (system_mode) {
            case TRIG_MODE:
                PORTD |= (1 << TRIG_PIN);
                _delay_us(10);
                PORTD &= ~(1 << TRIG_PIN);
                system_mode = REC_MODE;
                break;
            case SEND_MODE:
                distance = (tick_end - tick_start) * 0.544;
                angle = calculate_PID(distance);
                angle = (angle > MAX_ANGLE_SETTABLE) ? MAX_ANGLE_SETTABLE : (angle < MIN_ANGLE_SETTABLE)
                                                                            ? MIN_ANGLE_SETTABLE : angle;
                set_servo_angle((uint8_t) roundf(angle));
                system_mode = TRIG_MODE;

                if (DEBUG) {
                    char buf[64];
                    snprintf(buf, sizeof(buf), "Distance: %u cm, Angle: %u\n", (uint16_t) distance, (uint16_t) angle);
                    UART_putstring(buf);
                }
                _delay_ms(50);
                break;
            default:
                break;
        }
    }
}

// Clock Initialization Function
void clk_init(void) {
    CLKPR = (1 << CLKPCE); // Enable change clock bits
    CLKPR = (1 << CLKPS0); // Divide by 2 (8 MHz clock)
}

// ADC Initialization Function
void ADC_init(void) {
    ADMUX = (1 << REFS0); // AVcc with external capacitor at AREF pin
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // ADC Enable, Prescaler of 64
}

// Function to Read from ADC
float read_ADC(int channel, float max_value) {
    ADMUX = (ADMUX & 0xF8) | channel; // Select ADC channel with safety mask
    ADCSRA |= (1 << ADSC); // Start single conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC * (max_value / 1024.0); // Convert to float value scaled by max_value
}

// Initialize HC-SR04 Ultrasonic Sensor
void init_HC_SR04(void) {
    DDRD |= (1 << TRIG_PIN);
    PORTD &= ~(1 << TRIG_PIN);

    DDRB &= ~(1 << ECHO_PIN);
    PORTB &= ~(1 << ECHO_PIN);

    // Timer1 Configuration for Ultrasonic Measurement
    TCCR1B |= (1 << CS12);
    TCCR1A &= ~((1 << WGM10) | (1 << WGM11));
    TCCR1B &= ~((1 << WGM12) | (1 << WGM13));
    TCCR1B |= (1 << ICES1);
    TIFR1 |= (1 << ICF1);
    TIMSK1 |= (1 << ICIE1);
}

// Setup Servo PWM using Timer0
void setup_servo_PWM(void) {
    DDRD |= (1 << SERVO_PWM_PIN);

    // Timer0 Configuration for PWM
    TCCR0B |= (1 << CS00) | (1 << CS02);
    TCCR0A |= (1 << WGM00) | (1 << WGM01);
    TCCR0B |= (1 << WGM02);
    OCR0A = 155;
    TCCR0A |= (1 << COM0B1);
}

// Set Servo Angle
void set_servo_angle(uint8_t angle) {
    uint8_t pwm_value = 2 + (angle / 11.25);
    OCR0B = pwm_value;
}

// Calculate PID for Servo Control
float calculate_PID(float distance) {
    if (distance < 0 || distance > MAX_TRACK_LENGTH) return previous_angle;

    // Read PID parameters from analog pins
    float KP = read_ADC(KP_ADC_CHANNEL, MAX_KP_VALUE);
    float KI = read_ADC(KI_ADC_CHANNEL, MAX_KI_VALUE);
    float KD = read_ADC(KD_ADC_CHANNEL, MAX_KD_VALUE);

    if (DEBUG) {
        int intKP = (int) (KP * 100);
        int intKI = (int) (KI * 100);
        int intKD = (int) (KD * 100);

        char kpBuf[64], kiBuf[64], kdBuf[64];

        snprintf(kpBuf, sizeof(kpBuf), "KP: %d%%\n", intKP);
        snprintf(kiBuf, sizeof(kiBuf), "KI: %d%%\n", intKI);
        snprintf(kdBuf, sizeof(kdBuf), "KD: %d%%\n", intKD);

        UART_putstring(kpBuf);
        UART_putstring(kiBuf);
        UART_putstring(kdBuf);
    }

    float error = (distance > MAX_TRACK_LENGTH ? MAX_TRACK_LENGTH : distance) - TARGET_POSITION;
    integral += error;
    integral = (integral > I_MAX) ? I_MAX : (integral < I_MIN) ? I_MIN : integral;
    float derivative = error - previous_error;
    float output = KP * error + KI * integral + KD * derivative;
    previous_error = error;

    float angle = LEVEL_ANGLE + (output / MAX_TRACK_LENGTH) * (MAX_ANGLE - LEVEL_ANGLE);
    angle = angle < MIN_ANGLE ? MIN_ANGLE : (angle > MAX_ANGLE ? MAX_ANGLE : angle);
    previous_angle = angle;
    return angle;
}

// Interrupt Service Routine for Timer1 Capture Event
ISR(TIMER1_CAPT_vect) {
        if (TCCR1B & (1 << ICES1)) {
            tick_start = TCNT1;
            _delay_us(10);
            TCCR1B &= ~(1 << ICES1);
        } else {
            tick_end = TCNT1;
            tick_end += (tick_start > tick_end) ? 65535 : 0;
            system_mode = SEND_MODE;
            TCCR1B |= (1 << ICES1);
        }
}
