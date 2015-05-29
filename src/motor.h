//
// Created by Aquilla Sherrock on 5/28/15.
//

#ifndef CUBENSIS_MOTOR_H
#define CUBENSIS_MOTOR_H

#include "Arduino.h"

class Motor
{
public:
    unsigned short throttle;
    bool isKill;


    Motor(uint8_t pin) {
        pinMode(pin, OUTPUT);
    }

    void setThrottle(unsigned short throttle) {

    }

    int kill() {

    };
};


#endif //CUBENSIS_MOTOR_H
