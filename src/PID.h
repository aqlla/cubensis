/* ************************************************************************** *
 * PID Controller 															  *
 * Aquilla Sherrock - 16 May 2015											  *
 * ************************************************************************** */

#ifndef __IT_PID__
#define __IT_PID__

#include "Arduino.h"
#include "itvec.h"

enum ControllerDirection { DIRECT, REVERSE };
enum ControllerMode { AUTOMATIC, MANUAL };

class PID
{
public:
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
    PID(it_float* input, it_float* output, it_float* setpoint, it_float kp, it_float ki, it_float kd)
            :input(input), output(output), setpoint(setpoint)
    {
        samplePeriod = 0;
        setOutputLimits(-255, 255);

        setMode(AUTOMATIC);
        setDirection(DIRECT);

        setTuningCoefficients(kp, ki, kd);
        previousTime = millis() - samplePeriod;

        integralError = 0;
        previousInput = *setpoint;
    }

    /**
     * Compute new PID value.
     *
     * Checks elapsed time since last computation and runs again if enough time
     * has passed.
     * @return true if computation ran, false if not.
     */
    bool computeError()
    {
        if (mode == AUTOMATIC) {
            unsigned long now = millis();
            unsigned long dt = now - previousTime;

            if (dt >= samplePeriod) {
                // Calculate proportional error.
                it_float input = *(this->input);
                it_float proportionalError = *(this->setpoint) - input;

                // Calculate current integral error and add it to the accumulator.
                integralError += (k_integral * proportionalError) * dt;
                // getBoundedValue(&integralError);

                // Calculate derivative error
                it_float derivativeError = input - previousInput;

                // Calculate adjusted values considering the coefficients.
                it_float proportionalComponent = k_proportional * proportionalError;
                it_float derivativeComponent   = (k_derivative * derivativeError) / dt;

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
    void setTuningCoefficients(it_float kp, it_float ki, it_float kd)
    {
        this->k_proportional = kp;
        this->k_integral     = ki;
        this->k_derivative   = kd;

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
    void setOutputLimits(it_float min, it_float max)
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
    void setSamplePeriod(unsigned long period)
    {
        if (period > 0) {
            samplePeriod = period;
        }
    }

    /**
     * Set Direction.
     *
     * @param direction.
     */
    void setDirection(ControllerDirection direction)
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
    void setMode(ControllerMode mode)
    {
        if (mode != this->mode) {
            init();
        }

        this->mode = mode;
    }

private:
    it_float* input;
    it_float* output;
    it_float* setpoint;

    it_float k_proportional;	// Proportional tuning parameter.
    it_float k_integral;		// Integral tuning parameter.
    it_float k_derivative;	// Derivative tuning parameter.

    it_float integralError;
    it_float previousInput;
    it_float outputMin;
    it_float outputMax;

    unsigned long samplePeriod;
    unsigned long previousTime;

    ControllerDirection direction;
    ControllerMode mode;

    /**
     * Check Error Bounds.
     * Make sure error is within minimum and maximum output parameters.
     *
     * @param error: the error value.
     * @return value inside output min/max.
     */
    it_float getBoundedValue(it_float error)
    {
        if (error > outputMax) {
            return outputMax;
        } else if (error < outputMin) {
            return outputMin;
        }

        return error;
    }

    it_float getBoundedValue(it_float* error)
    {
        if (*error > outputMax) {
            *error = outputMax;
        } else if (*error < outputMin) {
            *error = outputMin;
        }

        return *error;
    }

    /**
     * Initialize/reset all state variables.
     */
    void init()
    {
        previousInput = *input;
        integralError = 0;
        // getBoundedValue(&integralError);
    }
};

#endif