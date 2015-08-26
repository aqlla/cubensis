//
// Created by Aquilla Sherrock on 5/28/15.
//

#ifndef CUBENSIS_MOTOR_H
#define CUBENSIS_MOTOR_H

#include "../lib/hardware/arduino/avr/variants/standard/pins_arduino.h"
#include "util.h"
#include "itvec.h"
#include "Servo.h"

#define MOTOR1_PIN 3
#define MOTOR2_PIN 5
#define MOTOR3_PIN 6
#define MOTOR4_PIN 9

#if defined(THROTTLE_POT)
    #define THROTTLE_PIN A3
    #define THROTTLE_SIGNAL_MIN 0
    #define THROTTLE_SIGNAL_MAX 1023
    #define THROTTLE_READ(x) analogRead(x)
#else
    #define THROTTLE_PIN CH3
    #define THROTTLE_SIGNAL_MIN CH3_PULSEIN_MIN
    #define THROTTLE_SIGNAL_MAX CH3_PULSEIN_MAX
    #define THROTTLE_READ(x) pulseIn(x, HIGH, 25000)
#endif

#define THROTTLE_MIN 10
#define THROTTLE_MAX 125
#define THROTTLE_KILL 10
#define MOTOR_START_THROTTLE1 10
#define MOTOR_START_THROTTLE2 20



class Motor
{
public:
    static bool killed;
    uint8_t throttle;

    Motor(uint8_t pin);
    void init();
    uint8_t set();
    uint8_t set_error(const cfloat);
    static void kill(const bool);
    static uint8_t read_throttle_pin();

private:
    Servo servo;
    uint8_t servoPin;
    static uint8_t throttlePinValue;
    static uint8_t get_mapped_throttle_value();
};


#endif //CUBENSIS_MOTOR_H
