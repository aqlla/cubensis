#ifndef __CUBENSIS_H__
#define __CUBENSIS_H__

#include "util.h"
#include "pid.h"
#include "imu.h"
#include "motor.h"

#define DBG_NONE            0
#define DBG_READABLE        1
#define DBG_ARSILISCOPE     2
#define CUBENSIS_DBG        DBG_ARSILISCOPE

#if defined(Arduino_h)
    #if CUBENSIS_DBG==DBG_NONE
    #define CUBE_PRINT(x)   ;
    #define CUBE_PRINTLN(x) ;
    #define PRINT_DELAY     0
    #else
    #define START_SERIAL(x) Serial.begin(x);
    #define CUBE_PRINT(x)   Serial.print(x)
    #define CUBE_PRINTLN(x) Serial.print(x); Serial.print("\n")
    #endif

    #if CUBENSIS_DBG==DBG_ARSILISCOPE
    #define PRINT_DELAY     100
    #elif CUBENSIS_DBG==DBG_READABLE
    #define PRINT_DELAY     450
    #endif
#endif


#define USE_KILL_SWITCH true
#define KILL_SIGNAL HIGH
#define KILL_PIN 2

class Cubensis {
public:
    enum class Status
    {
        KILL = -1,
        SLEEP,
        READY,
        RUNNING,
        ERROR
    };

    Cubensis();

    void start_motors();
    void calibrate_sensors(unsigned long);
    void start();
    void update();
    void kill();
    void print();
    Status check_status();

private:
    IMU* imu1;
    IMU* imu2;

    it::vec3<cfloat> orientation;
    it::vec3<cfloat> rotationRate;

    PID* pid_ratex;
    PID* pid_stabx;

    cfloat error_ratex;
    cfloat error_stabx;
    cfloat setpoint_ratex;
    cfloat setpoint_stabx;

    int throttle;
    Motor motor1;
    Motor motor2;
    Motor motor3;
    Motor motor4;

    Status status;
    unsigned long lastPrint;
    unsigned long now;
};

#endif
