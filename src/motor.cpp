//
// Created by Aquilla Sherrock on 5/29/15.
//

#include "motor.h"

int Motor::throttlePinValue = 0;

Motor::Motor(uint8_t pin)
    : is_kill(false),
      servoPin(pin),
      throttle(0)
{}

void Motor::init() {
    servo.attach(servoPin);
    servo.write(MOTOR_START_THROTTLE1);
}

int Motor::set() {
    throttle = (int) map(analogRead(THROTTLE_PIN), 0, 1023, THROTTLE_MIN, THROTTLE_MAX);
    servo.write(throttle);
    return throttle;
}

int Motor::set(int error) {
    if (is_kill) return -1;

    throttle = throttlePinValue + error;
    if (throttle > THROTTLE_MAX) {
        throttle = THROTTLE_MAX;
    } else if (throttle < THROTTLE_MIN) {
        throttle = THROTTLE_MIN;
    }

    servo.write(throttle);
    return throttle;
}

int Motor::getThrottle() {
    return servo.read();
}

void Motor::kill(bool doKill) {
    throttle = THROTTLE_KILL;
    is_kill = doKill;
}

int Motor::getThrottlePinValue() {
    throttlePinValue = (int) map(analogRead(THROTTLE_PIN), 0, 1023, THROTTLE_MIN, THROTTLE_MAX);
    return throttlePinValue;
}