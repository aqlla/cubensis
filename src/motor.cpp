//
// Created by Aquilla Sherrock on 5/29/15.
//

#include "motor.h"

uint8_t Motor::throttlePinValue = 0;
bool Motor::killed = false;


Motor::Motor(uint8_t pin)
    : servoPin{pin},
      throttle{0}
{}

void Motor::init() {
    servo.attach(servoPin);
    servo.write(MOTOR_START_THROTTLE1);
}

uint8_t Motor::set() {
    if (killed) {
        throttle = 0;
        return 0;
    }

    throttle = get_mapped_throttle_value();
    servo.write(throttle);
    return throttle;
}

uint8_t Motor::set_error(const cfloat error) {
    if (killed) {
        throttle = 0;
        return 0;
    }

    int16_t adjustedThrottle = throttlePinValue + (int16_t) error;

    if (adjustedThrottle > THROTTLE_MAX)
        throttle = THROTTLE_MAX;
    else if (adjustedThrottle < THROTTLE_MIN)
        throttle = THROTTLE_MIN;
    else
        throttle = (uint8_t) adjustedThrottle;

    servo.write(throttle);
    return throttle;
}

void Motor::kill(const bool do_kill) {
    killed = do_kill;
    if (do_kill)
        throttlePinValue = THROTTLE_KILL;
    else
        read_throttle_pin();

}

uint8_t Motor::read_throttle_pin() {
    return throttlePinValue = get_mapped_throttle_value();
}

uint8_t Motor::get_mapped_throttle_value() {
    return (uint8_t) map(THROTTLE_READ(THROTTLE_PIN), THROTTLE_SIGNAL_MIN, THROTTLE_SIGNAL_MAX, THROTTLE_MIN, THROTTLE_MAX);
}