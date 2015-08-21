#ifndef __CUBENSIS_H__
#define __CUBENSIS_H__

#include "Arduino.h"
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
#define KILL_SIGNAL LOW
#define KILL_PIN 2

enum class CubensisStatus
{
    KILL = -1,
    SLEEP,
    READY,
    RUNNING,
    ERROR
};

class Cubensis {
public:
    Cubensis();

    void prepare();
    void calibrateSensors(unsigned long calibrationTime);
    void start();
    void update();
    void kill();
    void print();
    CubensisStatus check_status();

private:
    IMU* imu1;
    IMU* imu2;

    ITVec3<it_float> orientation;
    ITVec3<it_float> rotationRate;

    PID* pid_ratex;
    PID* pid_stabx;
    it_float error_ratex;
    it_float error_stabx;
    it_float setpoint_ratex;
    it_float setpoint_stabx;

    Motor motor1;
    Motor motor2;
    Motor motor3;
    Motor motor4;

    int throttle;

    CubensisStatus status;
    unsigned long lastPrint;
    unsigned long now;
};

#endif
