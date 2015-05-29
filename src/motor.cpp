//
// Created by Aquilla Sherrock on 5/29/15.
//

#include "motor.h"

Motor::Motor(uint8_t pin) {
    servo.attach(pin);
}

void Motor::setThrottle(uint8_t rate_error=0) {
    if (!is_kill) {
        throttle = throttleAll + rate_error;
        if (throttle > THROTTLE_MAX) {
            throttle = THROTTLE_MAX;
        } else if (throttle < THROTTLE_MIN) {
            throttle = THROTTLE_MIN;
        }

        servo.write(throttle);
    } else {
        servo.write(THROTTLE_KILL);
    }
};

void Motor::start() {
    setThrottle(MOTOR_START_THROTTLE1);
}

static void Motor::setThrottleAll(uint8_t value) {
    throttleAll = value;
}

static void Motor::kill(bool doKill=true) {
    throttleAll = THROTTLE_KILL;
    is_kill = doKill;
}