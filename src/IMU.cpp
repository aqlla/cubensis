// Created by Aquilla Sherrock on 5/30/15.
// Copyright (c) 2015 Insignificant Tech. All rights reserved.

#include "imu.h"

IMU::IMU(Address address)
        :address{address},
         status{Status::SLEEP},
         LSB_PER_DEG_PER_SEC{131},
         LSB_PER_G{16384}

{
    device = new MPU6050{};
    gyroData  = new it::vec3<int16_t>{};
    accelData = new it::vec3<int16_t>{};
    rotation  = new it::vec3<cfloat>{};
    gyroOffset    = new it::vec3<cfloat>{};
    accelOffset   = new it::vec3<cfloat>{};
    acceleration  = new it::AccelerationVec{};
    orientation   = new it::vec3<cfloat>{};
    complementary = new it::vec3<cfloat>{};

    if (address != Address::AD0_LO && address != Address::AD0_HI) {
        status = Status::ADDRESS_ERROR;
    } else {
        device->initialize();
        if (device->testConnection()) {
            status = Status::OK;
        } else {
            status = Status::CONNECTION_ERROR;
        }
    }
};


IMU::~IMU()
{
    delete device;
    delete rotation;
    delete gyroData;
    delete accelData;
    delete gyroOffset;
    delete accelOffset;
    delete orientation;
    delete acceleration;
    delete complementary;
};


/**
 * Set gyroscope sensitivity.
 */
void IMU::setGyroscopeSensitivity(uint8_t sensitivity)
{
    if (sensitivity < 0 || sensitivity >= 4) {
        sensitivity = 0;
    }

    device->setFullScaleGyroRange(sensitivity);
    LSB_PER_DEG_PER_SEC = IMU::GYR_SENSITIVITIES[sensitivity];
}



/**
 * Set accelerometer sensitivity.
 */
void IMU::setAccelerometerSensitivity(uint8_t sensitivity)
{
    if (sensitivity < 0 || sensitivity >= 4) {
        sensitivity = 0;
    }

    device->setFullScaleAccelRange(sensitivity);
    LSB_PER_G = IMU::ACC_SENSITIVITIES[sensitivity];
}



bool IMU::ok() {
    return status == Status::OK;
}



/**
 * Update Orientation.
 * Get the values from the IMU and update the current axial orientation
 * values based on gyroscope readings, calculates the roll and pitch from
 * accelerometer data, and combines these values with a complementary
 * filter.
 */
void IMU::updateOrientation()
{
    getAccelerationAndRotation();

    short zDirection = getUpOrDown();
    orientation->x += rotation->x * zDirection;
    orientation->y += rotation->y * zDirection;
    orientation->z += rotation->z;

    flipAngle(&orientation->x);
    flipAngle(&orientation->y);
    flipAngle(&orientation->z);

    complementary->x = ALPHA * orientation->x + (1 - ALPHA) * acceleration->roll();
    complementary->y = ALPHA * orientation->y + (1 - ALPHA) * acceleration->pitch();
    complementary->z = orientation->z;
}



/**
 * Calibrate the IMU device.
 *
 * Measures the average accelerometer and gyroscope data over a given
 * number of iterations (preferably when the device is steady and the
 * z-azis is normal to gravity) and accordingly sets offset values per axis.
 *
 * @param iterations: number of times to scan IMU readings.
 */
void IMU::calibrate(unsigned long time)
{
    unsigned long startTime = TIME_FUNC();
    int iterations = 0;

    while (TIME_FUNC() - startTime < time) {
        device->getMotion6(&accelData->x, &accelData->y, &accelData->z, &gyroData->x, &gyroData->y, &gyroData->z);
        *accelOffset += *accelData / LSB_PER_G;             // convert to G's
        *gyroOffset += *gyroData / LSB_PER_DEG_PER_SEC;     // convert to deg/sec
        iterations++;
    }

    *accelOffset /= (cfloat) iterations;
    *gyroOffset  /= (cfloat) iterations;
    accelOffset->z -= 1;    // Account for gravity
}



/**
 * Initialize the previous time value;
 */
void IMU::startTime() {
    previousTime = TIME_FUNC();
}



/**
    * Read in new IMU values.
    *
    * Get raw acceleration and gyroscopic rate data from IMU and convert to
    * mg & deg/s values respectively.
    */
void IMU::getAccelerationAndRotation()
{
    unsigned long currentTime = TIME_FUNC();
    cfloat dt = (currentTime - previousTime) / TIME_TO_SEC;
    previousTime = currentTime;

    device->getMotion6(&accelData->x, &accelData->y, &accelData->z, &gyroData->x, &gyroData->y, &gyroData->z);
    *rotation = 0.0 - (*gyroData / LSB_PER_DEG_PER_SEC - *gyroOffset) * dt;
    *acceleration = *accelData / LSB_PER_G - *accelOffset * 9.81;
}



void IMU::flipAngle(cfloat*angle)
{
    if (*angle > 180) {
        *angle = -180 - (*angle - 180);
    } else if (*angle < -180) {
        *angle = 180 + (*angle + 180);
    }
}



short IMU::getUpOrDown()
{
    return acceleration->z > 0
           ? (short) 1
           : (short) -1;
}