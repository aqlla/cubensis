//
// Created by Aquilla Sherrock on 5/29/15.
//

#include "motor.h"

bool Motor::is_kill = false;
uint8_t Motor::throttle = 0;


Motor::Motor(uint8_t pin) {
    servo.attach(pin);
}

void Motor::setError(it_float rate_error) {
    if (!is_kill) {
        error = throttle + rate_error;
        if (error > THROTTLE_MAX) {
            error = THROTTLE_MAX;
        } else if (error < THROTTLE_MIN) {
            error = THROTTLE_MIN;
        }

        servo.write(error);
    } else {
        servo.write(THROTTLE_KILL);
    }
};

void Motor::start() {
    if (!is_kill) {
        error = MOTOR_START_THROTTLE1;
        servo.write(error);
    }
}

uint8_t Motor::getThrottle() {
    return throttle + error;
}

void Motor::setThrottle(uint8_t value) {
    throttle = value;
}

void Motor::setThrottle() {
    throttle = map(analogRead(THROTTLE_PIN), 0, 1023, THROTTLE_MIN, THROTTLE_MAX);
}

void Motor::kill(bool doKill) {
    throttle = THROTTLE_KILL;
    is_kill = doKill;
}