#ifndef __CUBENSIS_H__
#define __CUBENSIS_H__

#include "Arduino.h"
#include "Servo.h"
//#include "ServoTimers.h"
#include "itvec.h"
#include "pid.h"
#include "imu.h"

#define CUBENSIS_DEBUG


#define IMU1_ADDR MPU6050_ADDRESS_AD0_LOW
#define IMU2_ADDR MPU6050_ADDRESS_AD0_HIGH

#define CUBENSIS_STATUS_KILL 		   -1
#define CUBENSIS_STATUS_SLEEP 			0
#define CUBENSIS_STATUS_RUNNING 		1
#define CUBENSIS_STATUS_ERROR_IMU1 		2
#define CUBENSIS_STATUS_ERROR_IMU2 		3
#define CUBENSIS_STATUS_ERROR_BOTH_IMU  4

#define KILL_PIN_KILL_SIGNAL LOW
#define THROTTLE_PIN 2
#define KILL_PIN 2

#define MOTOR1_PIN 3
#define MOTOR2_PIN 5
#define MOTOR3_PIN 6
#define MOTOR4_PIN 9

#define THROTTLE_MIN 10
#define THROTTLE_MAX 125
#define THROTTLE_KILL 10
#define MOTOR_START_THROTTLE1 10
#define MOTOR_START_THROTTLE2 20

class Cubensis {
public:
    IMU* imu1;
    IMU* imu2;

    short status;
    ITVec3<it_float>* orientation;
    ITVec3<it_float>* rotationRate;

    PID* pidx;
    it_float rate_error;
    it_float setPoint;

    Servo motor1;
    Servo motor2;
    Servo motor3;
    Servo motor4;

    int motor1_throt;
    int motor2_throt;
    int motor3_throt;
    int motor4_throt;

    int throttle;

    Cubensis() {
        status = CUBENSIS_STATUS_SLEEP;
        imu1 = new IMU(IMU1_ADDR);
        imu2 = new IMU(IMU2_ADDR);

        rate_error = 0;
        setPoint = 0;
        orientation = new ITVec3<it_float>();
        rotationRate = new ITVec3<it_float>();
        pidx = new PID(&rotationRate->x, &rate_error, &setPoint, 100, 0, 15);

        pinMode(KILL_PIN, INPUT);
        motor1.attach(MOTOR1_PIN);
        motor2.attach(MOTOR2_PIN);
        motor3.attach(MOTOR3_PIN);
        motor4.attach(MOTOR4_PIN);

        bool imu1Status = imu1->init();
        bool imu2Status = imu2->init();

        if (imu1Status == IMU_STATUS_OK && imu2Status == IMU_STATUS_OK) {
            status = CUBENSIS_STATUS_RUNNING;
        } else if (imu1Status != IMU_STATUS_OK && imu2Status != IMU_STATUS_OK) {
            status = CUBENSIS_STATUS_ERROR_BOTH_IMU;
        } else if (imu1Status == IMU_STATUS_OK) {
            status = CUBENSIS_STATUS_ERROR_IMU2;
        } else {
            status = CUBENSIS_STATUS_ERROR_IMU1;
        }
    };


    void startMotors1() {
        motor1.write(MOTOR_START_THROTTLE1);
        motor2.write(MOTOR_START_THROTTLE1);
        motor3.write(MOTOR_START_THROTTLE1);
        motor4.write(MOTOR_START_THROTTLE1);
        motor1_throt = MOTOR_START_THROTTLE1;
        motor2_throt = MOTOR_START_THROTTLE1;
        motor3_throt = MOTOR_START_THROTTLE1;
        motor4_throt = MOTOR_START_THROTTLE1;

        #ifdef CUBENSIS_DEBUG
        arsiliscope();
        #endif
    };


    void startMotors2() {
        motor1.write(MOTOR_START_THROTTLE2);
        motor2.write(MOTOR_START_THROTTLE2);
        motor3.write(MOTOR_START_THROTTLE2);
        motor4.write(MOTOR_START_THROTTLE2);
        motor1_throt = MOTOR_START_THROTTLE2;
        motor2_throt = MOTOR_START_THROTTLE2;
        motor3_throt = MOTOR_START_THROTTLE2;
        motor4_throt = MOTOR_START_THROTTLE2;

        #ifdef CUBENSIS_DEBUG
        arsiliscope();
        #endif
    };

    void calibrate(short iterations) {
        imu1->calibrate(iterations);
        imu2->calibrate(iterations);
    };

    void start() {
        imu1->startTime();
        imu2->startTime();
        update();
    };


    void update() {
        if (digitalRead(KILL_PIN) == KILL_PIN_KILL_SIGNAL) {
            kill();
        } else if (status != CUBENSIS_STATUS_KILL) {
            imu1->updateOrientation();
            imu2->updateOrientation();

            orientation->x = (imu1->complementaryAngle->x + imu2->complementaryAngle->x) / 2.0;
            orientation->y = (imu1->complementaryAngle->y + imu2->complementaryAngle->y) / 2.0;
            orientation->z = (imu1->complementaryAngle->z + imu2->complementaryAngle->z) / 2.0;

            rotationRate->x = (imu1->rotation->x + imu2->rotation->x) / 2.0;
            rotationRate->y = (imu1->rotation->y + imu2->rotation->y) / 2.0;
            rotationRate->z = (imu1->rotation->z + imu2->rotation->z) / 2.0;
            pidx->compute();

            throttle = (int) map(analogRead(THROTTLE_PIN), 0, 1023, THROTTLE_MIN, THROTTLE_MAX);

            motor1_throt = throttle;
            motor2_throt = max(min(throttle - rate_error, THROTTLE_MAX), THROTTLE_MIN);
            motor3_throt = throttle;
            motor4_throt = max(min(throttle + rate_error, THROTTLE_MAX), THROTTLE_MIN);

            motor1.write(motor1_throt);
            motor2.write(motor2_throt);
            motor3.write(motor3_throt);
            motor4.write(motor4_throt);
        }

        if (status == CUBENSIS_STATUS_KILL && digitalRead(KILL_PIN) != KILL_PIN_KILL_SIGNAL) {
            status = CUBENSIS_STATUS_RUNNING;
        }
    };

    void kill() {
        throttle = 0;
        motor1_throt = THROTTLE_KILL;
        motor2_throt = THROTTLE_KILL;
        motor3_throt = THROTTLE_KILL;
        motor4_throt = THROTTLE_KILL;

        motor1.write(THROTTLE_KILL);
        motor2.write(THROTTLE_KILL);
        motor3.write(THROTTLE_KILL);
        motor4.write(THROTTLE_KILL);

        status = CUBENSIS_STATUS_KILL;
    };

    void arsiliscope() {
        Serial.print("{\"roll\": ");
        Serial.print(orientation->x);
        Serial.print(",\"pitch\": ");
        Serial.print(rotationRate->x);
        Serial.print(",\"yaw\": ");
        Serial.print(rate_error);
        Serial.print(",\"motor1\": ");
        Serial.print(motor1_throt);
        Serial.print(",\"motor2\": ");
        Serial.print(motor2_throt);
        Serial.print(",\"motor3\": ");
        Serial.print(motor3_throt);
        Serial.print(",\"motor4\": ");
        Serial.print(motor4_throt);
        Serial.println("}");
    };
};

#endif