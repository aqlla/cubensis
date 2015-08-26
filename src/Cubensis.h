#ifndef __CUBENSIS_H__
#define __CUBENSIS_H__

#include "util.h"
#include "pid.h"
#include "imu.h"
#include "motor.h"



#define MOTOR_COUNT 4

#define USE_KILL_SWITCH true
#define KILL_SIGNAL HIGH
#define KILL_PIN 2
#define STATUS1_LED_PIN 13
#define STATUS2_LED_PIN 12

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
    Status check_status();

private:
    IMU* imu1;
    IMU* imu2;

    it::vec3<cfloat> orientation;
    it::vec3<cfloat> rotationRate;

    it::vec3<PID*> pid_rate;
    it::vec3<PID*> pid_stab;
    it::vec3<cfloat> error_rate;
    it::vec3<cfloat> error_stab;
    it::vec3<cfloat> setpoint_rate;
    it::vec3<cfloat> setpoint_stab;

    Motor motors[MOTOR_COUNT];

    Status status;
    unsigned long lastPrint;
    unsigned long now;

    void kill();
    void print();
};

#endif
