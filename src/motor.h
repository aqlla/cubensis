//
// Created by Aquilla Sherrock on 5/28/15.
//

#ifndef CUBENSIS_MOTOR_H
#define CUBENSIS_MOTOR_H

#include "Arduino.h"
#include "itvec.h"
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
    Motor(uint8_t pin);

    void start();
    void setError(it_float=0);
    uint8_t getThrottle();
    static void setThrottle(uint8_t);
    static void setThrottle();
    static void kill(bool=true);

private:
    Servo servo;
    uint8_t pin;

    uint8_t error;
    static uint8_t throttle;
    static bool is_kill;
};


#endif //CUBENSIS_MOTOR_H
