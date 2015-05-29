//
// Created by Aquilla Sherrock on 5/29/15.
//

#include "Cubensis.h"

Cubensis::Cubensis()
{
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
}


void Cubensis::startMotors1()
{
    motor1.write(MOTOR_START_THROTTLE1);
    motor2.write(MOTOR_START_THROTTLE1);
    motor3.write(MOTOR_START_THROTTLE1);
    motor4.write(MOTOR_START_THROTTLE1);
    motor1_throt = MOTOR_START_THROTTLE1;
    motor2_throt = MOTOR_START_THROTTLE1;
    motor3_throt = MOTOR_START_THROTTLE1;
    motor4_throt = MOTOR_START_THROTTLE1;

    #if CUBENSIS_DBG!=DBG_NONE
    print();
    #endif
}


void Cubensis::startMotors2()
{
    motor1.write(MOTOR_START_THROTTLE2);
    motor2.write(MOTOR_START_THROTTLE2);
    motor3.write(MOTOR_START_THROTTLE2);
    motor4.write(MOTOR_START_THROTTLE2);
    motor1_throt = MOTOR_START_THROTTLE2;
    motor2_throt = MOTOR_START_THROTTLE2;
    motor3_throt = MOTOR_START_THROTTLE2;
    motor4_throt = MOTOR_START_THROTTLE2;

    #if CUBENSIS_DBG!=DBG_NONE
    print();
    #endif
}

void Cubensis::calibrate(short iterations)
{
    imu1->calibrate(iterations);
    imu2->calibrate(iterations);
}

void Cubensis::start()
{
    imu1->startTime();
    imu2->startTime();
    update();
}


void Cubensis::update()
{
    if (digitalRead(KILL_PIN) == KILL_SIGNAL) {
        kill();
    } else if (status != CUBENSIS_STATUS_KILL) {
        imu1->updateOrientation();
        imu2->updateOrientation();

        orientation->x = (imu1->complementary->x + imu2->complementary->x) / 2.0;
        orientation->y = (imu1->complementary->y + imu2->complementary->y) / 2.0;
        orientation->z = (imu1->complementary->z + imu2->complementary->z) / 2.0;

        rotationRate->x = (imu1->rotation->x + imu2->rotation->x) / 2.0;
        rotationRate->y = (imu1->rotation->y + imu2->rotation->y) / 2.0;
        rotationRate->z = (imu1->rotation->z + imu2->rotation->z) / 2.0;
        pidx->compute();

        throttle = map_uint8(analogRead(THROTTLE_PIN), 0, 1023, THROTTLE_MIN, THROTTLE_MAX);

        motor1_throt = throttle;
        motor2_throt = max(min(throttle - rate_error, THROTTLE_MAX), THROTTLE_MIN);
        motor3_throt = throttle;
        motor4_throt = max(min(throttle + rate_error, THROTTLE_MAX), THROTTLE_MIN);

        motor1.write(motor1_throt);
        motor2.write(motor2_throt);
        motor3.write(motor3_throt);
        motor4.write(motor4_throt);
    }

    if (status == CUBENSIS_STATUS_KILL && digitalRead(KILL_PIN) != KILL_SIGNAL) {
        status = CUBENSIS_STATUS_RUNNING;
    }
}

void Cubensis::kill()
{
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
}

int Cubensis::map_uint8(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Cubensis::print()
{
    #if CUBENSIS_DBG==DBG_ARSILISCOPE
        CUBE_PRINT("{\"roll\": ");
        CUBE_PRINT(orientation->x);
        CUBE_PRINT(",\"pitch\": ");
        CUBE_PRINT(rotationRate->x);
        CUBE_PRINT(",\"yaw\": ");
        CUBE_PRINT(rate_error);
        CUBE_PRINT(",\"motor1\": ");
        CUBE_PRINT(motor1_throt);
        CUBE_PRINT(",\"motor2\": ");
        CUBE_PRINT(motor2_throt);
        CUBE_PRINT(",\"motor3\": ");
        CUBE_PRINT(motor3_throt);
        CUBE_PRINT(",\"motor4\": ");
        CUBE_PRINT(motor4_throt);
        CUBE_PRINTLN("}");
    #elif CUBENSIS_DBG==DBG_READABLE

    #endif
}