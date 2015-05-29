//
// Created by Aquilla Sherrock on 5/29/15.
//

#include "Cubensis.h"

Cubensis::Cubensis()
{
    status = CUBENSIS_STATUS_SLEEP;
    imu1 = new IMU(IMU1_ADDR);
    imu2 = new IMU(IMU2_ADDR);
    bool imu1Status = imu1->init();
    bool imu2Status = imu2->init();

    if (imu1Status != IMU_STATUS_OK || imu2Status != IMU_STATUS_OK) {
        status = CUBENSIS_STATUS_ERROR_BOTH_IMU;

        if (imu1Status == IMU_STATUS_OK) {
            status = CUBENSIS_STATUS_ERROR_IMU2;
        } else if (imu2Status == IMU_STATUS_OK) {
            status = CUBENSIS_STATUS_ERROR_IMU1;
        }
    } else {
        setPoint = 0;
        rate_error = 0;
        orientation = new ITVec3<it_float>();
        rotationRate = new ITVec3<it_float>();
        pidx = new PID(&rotationRate->x, &rate_error, &setPoint, 100, 0, 15);

        pinMode(KILL_PIN, INPUT);

        motor1 = new Motor(MOTOR1_PIN);
        motor2 = new Motor(MOTOR2_PIN);
        motor3 = new Motor(MOTOR3_PIN);
        motor4 = new Motor(MOTOR4_PIN);
    }


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


void Cubensis::startMotors()
{
    motor1->start();
    motor2->start();
    motor3->start();
    motor4->start();
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
    if (status != CUBENSIS_STATUS_KILL) {
        if (digitalRead(KILL_PIN) == KILL_SIGNAL) {
            status = CUBENSIS_STATUS_KILL;
            Motor::kill();
            return;
        }

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

        Motor::setThrottleAll(throttle);
        motor2->setThrottle(rate_error);
        motor4->setThrottle(-rate_error);
        // motor1->setThrottle();
        // motor3->setThrottle();

    } else if (digitalRead(KILL_PIN) != KILL_SIGNAL) {
        Motor::kill(false);
        status = CUBENSIS_STATUS_RUNNING;
    }
}

uint8_t Cubensis::map_uint8(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (uint8_t) (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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