//
// Created by Aquilla Sherrock on 5/28/15.
//

#ifndef CUBENSIS_MOTOR_H
#define CUBENSIS_MOTOR_H

#include "Arduino.h"
#include "Servo.h"

#define MOTOR1_PIN 3
#define MOTOR2_PIN 5
#define MOTOR3_PIN 6
#define MOTOR4_PIN 9

#define THROTTLE_PIN 2
#define THROTTLE_MIN 10
#define THROTTLE_MAX 125
#define THROTTLE_KILL 10
#define MOTOR_START_THROTTLE1 10
#define MOTOR_START_THROTTLE2 20



class Motor
{
public:
    Servo servo;
    uint8_t pin;

    uint8_t throttle;
    static uint8_t throttleAll;
    static bool is_kill;

    Motor(uint8_t pin);
    void start();
    void setThrottle(uint8_t rate_error=0);
    static void setThrottleAll(uint8_t value);
    static void kill(bool=true);

private:

};


#endif //CUBENSIS_MOTOR_H
