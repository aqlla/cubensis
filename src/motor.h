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
    static int throttlePinValue;

    Motor(uint8_t pin);
    void init();
    int set();
    int set(int);
//    static int setAll();
//    static int setAll(int);
    int getThrottle();
    void kill(bool=true);
    static int getThrottlePinValue();

private:
    Servo servo;
    uint8_t servoPin;
    int throttle;
    bool is_kill;
};


#endif //CUBENSIS_MOTOR_H
