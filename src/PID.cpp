// Created by Aquilla Sherrock on 5/30/15.
// Copyright (c) 2015 Insignificant Tech. All rights reserved.

#include "pid.h"

/**
 * PID Constructor.
 *
 * User passes references to their input, output, and setpoint variables so
 * they may be more easily accessed by the PID object.
 *
 * @param input: input data, probably the value of a pin read on Arduino.
 * @param output: error data the user will use from PID.
 * @param setpoint: the correct point. where error is 0.
 * @param kp: Proportional tuning parameter.
 * @param ki: Integral tuning parameter.
 * @param kd: Derivative tuning parameter.
 */
PID::PID(cfloat* input, cfloat* output, cfloat* setpoint, cfloat kp, cfloat ki, cfloat kd)
    :input(input),
     output(output),
     setpoint(setpoint),
     samplePeriod(0),
     integralError(0),
     previousInput(*setpoint)
{
    setMode(AUTOMATIC);
    setDirection(DIRECT);
    setOutputLimits(-255, 255);
    setTuningCoefficients(kp, ki, kd);
    previousTime = millis();
}



/**
 * Compute new PID value.
 *
 * Checks elapsed time since last computation and runs again if enough time
 * has passed.
 * @return true if computation ran, false if not.
 */
bool PID::computeError()
{
    if (mode == AUTOMATIC) {
        unsigned long now = millis();
        unsigned long dt = now - previousTime;

        if (dt >= samplePeriod) {
            // Calculate proportional error.
            cfloat input = *(this->input);
            cfloat proportionalError = *(this->setpoint) - input;

            // Calculate current integral error and add it to the accumulator.
            integralError += (k_integral * proportionalError) * dt;

            // Calculate derivative error
            cfloat derivativeError = input - previousInput;

            // Calculate adjusted values considering the coefficients.
            cfloat proportionalComponent = k_proportional * proportionalError;
            cfloat derivativeComponent   = k_derivative * derivativeError / dt;

            // Calculate new error output.
            *output = getBoundedValue(proportionalComponent + integralError - derivativeComponent);

            previousInput = input;
            previousTime  = now;
            return true;
        }
    }

    return false;
}



/**
 * Tune PID.
 *
 * @param kp: Proportional tuning parameter.
 * @param ki: Integral tuning parameter.
 * @param kd: Derivative tuning parameter.
 */
void PID::setTuningCoefficients(cfloat kp, cfloat ki, cfloat kd)
{
    k_proportional = kp;
    k_integral     = ki;
    k_derivative   = kd;

    if (direction == REVERSE) {
        k_proportional = 0 - k_proportional;
        k_integral     = 0 - k_integral;
        k_derivative   = 0 - k_derivative;
    }
}



/**
 * Set Output Limits.
 *
 * @param min: minimum output value.
 * @param max: maximum output value.
 */
void PID::setOutputLimits(cfloat min, cfloat max)
{
    if (min > max) return;
    outputMin = min;
    outputMax = max;
}

/**
 * Set Sample Period.
 *
 * @param period in milliseconds at witch samples will be taken.
 */
void PID::setSamplePeriod(unsigned long period)
{
    if (period > 0) samplePeriod = period;
}



/**
 * Set Direction.
 *
 * @param direction.
 */
void PID::setDirection(ControllerDirection direction)
{
    if (direction == REVERSE && direction != this->direction) {
        k_proportional = 0 - k_proportional;
        k_integral     = 0 - k_integral;
        k_derivative   = 0 - k_derivative;
    }

    this->direction = direction;
}



/**
 * Set Mode.
 *
 * @param mode.
 */
void PID::setMode(ControllerMode mode)
{
    if (mode != this->mode)
        init();

    this->mode = mode;
}



/**
  * Check Error Bounds.
  * Make sure error is within minimum and maximum output parameters.
  *
  * @param error: the error value.
  * @return value inside output min/max.
  */
cfloat PID::getBoundedValue(cfloat error)
{
    if (error > outputMax) {
        return outputMax;
    } else if (error < outputMin) {
        return outputMin;
    }

    return error;
}



/**
 * Initialize/reset all state variables.
 */
void PID::init()
{
    previousInput = *input;
    integralError = 0;
    // getBoundedValue(&integralError);
}
