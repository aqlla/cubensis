/* ************************************************************************** *
 * PID Controller 															  *
 * Aquilla Sherrock - 16 May 2015											  *
 * ************************************************************************** */

#ifndef __IT_PID__
#define __IT_PID__

#include "util.h"

enum ControllerDirection { DIRECT, REVERSE };
enum ControllerMode { AUTOMATIC, MANUAL };

class PID
{
public:
    PID(cfloat*, cfloat*, cfloat*, cfloat, cfloat, cfloat);
    bool computeError();
    void setTuningCoefficients(cfloat, cfloat, cfloat);
    void setOutputLimits(cfloat, cfloat);
    void setSamplePeriod(unsigned long);
    void setDirection(ControllerDirection);
    void setMode(ControllerMode);

private:
    cfloat* input;
    cfloat* output;
    cfloat* setpoint;

    cfloat k_proportional;	// Proportional tuning parameter.
    cfloat k_integral;		// Integral tuning parameter.
    cfloat k_derivative;	// Derivative tuning parameter.

    cfloat integralError;
    cfloat previousInput;
    cfloat outputMin;
    cfloat outputMax;

    unsigned long samplePeriod;
    unsigned long previousTime;

    ControllerDirection direction;
    ControllerMode mode;

    cfloat getBoundedValue(cfloat);
    void init();
};

#endif
