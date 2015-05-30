#ifndef __CUBENSIS_H__
#define __CUBENSIS_H__

#include "Arduino.h"
#include "Servo.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"

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


#define CUBENSIS_STATUS_KILL 		   -1
#define CUBENSIS_STATUS_SLEEP 			0
#define CUBENSIS_STATUS_RUNNING 		1
#define CUBENSIS_STATUS_ERROR_IMU1 		2
#define CUBENSIS_STATUS_ERROR_IMU2 		3
#define CUBENSIS_STATUS_ERROR_BOTH_IMU  4

#define IMU1_ADDR MPU6050_ADDRESS_AD0_LOW
#define IMU2_ADDR MPU6050_ADDRESS_AD0_HIGH

#define USE_KILL_SWITCH true
#define KILL_SIGNAL LOW
#define KILL_PIN 2
#define THROTTLE_PIN 2
#define THROTTLE_MIN 10
#define THROTTLE_MAX 125
#define THROTTLE_KILL 10

class Cubensis {
public:
    Cubensis();

    void startMotors();
    void calibrate(unsigned long timeToCalibrate);
    void start();
    void update();
    void print();

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

    Motor* motor1;
    Motor* motor2;
    Motor* motor3;
    Motor* motor4;

    uint8_t throttle;

    short status;
    static unsigned long lastPrint;
    static unsigned long now;
};

#endif