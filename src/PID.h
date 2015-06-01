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
    PID(it_float*, it_float*, it_float*, it_float, it_float, it_float);
    bool computeError();
    void setTuningCoefficients(it_float, it_float, it_float);
    void setOutputLimits(it_float, it_float);
    void setSamplePeriod(unsigned long);
    void setDirection(ControllerDirection);
    void setMode(ControllerMode);

private:
    it_float* input;
    it_float* output;
    it_float* setpoint;

    it_float k_proportional;	// Proportional tuning parameter.
    it_float k_integral;		// Integral tuning parameter.
    it_float k_derivative;	    // Derivative tuning parameter.

    it_float integralError;
    it_float previousInput;
    it_float outputMin;
    it_float outputMax;

    unsigned long samplePeriod;
    unsigned long previousTime;

    ControllerDirection direction;
    ControllerMode mode;

    it_float getBoundedValue(it_float);
    void init();
};

#endif
