//
// Created by Aquilla Sherrock on 5/16/15.
//

#ifndef CUBENSIS_IMU_H
#define CUBENSIS_IMU_H

#include "Arduino.h"
#include "MPU6050.h"
#include "itvec.h"

#define IMU_STATUS_SLEEP            0
#define IMU_STATUS_OK               1
#define IMU_STATUS_CONNECTION_ERROR 2
#define IMU_STATUS_ADDRESS_ERROR    3

#define IMU_ADDR_AD0_LOW  MPU6050_ADDRESS_AD0_LOW
#define IMU_ADDR_AD0_HIGH MPU6050_ADDRESS_AD0_HIGH

#define TIME_FUNC millis
#define TIME_TO_SEC 1000.0


class IMU
{
public:
    ITVec3<it_float> *rotation;
    ITVec3<it_float> *complementary;

    IMU(short address);
    ~IMU();

    void updateOrientation();
    void calibrate(unsigned long timeToCalibrate);
    void setGyroscopeSensitivity(uint8_t sensitivity);
    void setAccelerometerSensitivity(uint8_t sensitivity);
    void startTime();
    bool ok();

private:
    // The IMU device 
    MPU6050* device;
    short status;
    short address;
    unsigned long previousTime;

    ITVec3<it_float> *gyroOffset;
    ITVec3<it_float> *accelOffset;
    ITVec3<it_float> *orientation;
    AccelerationVec  *acceleration;


    // Alpha value for complimentary filter
    constexpr static it_float ALPHA = .965;

    /**
    * Raw Gyroscope readings.
    *
    * From i2cdev docs: http://www.i2cdevlib.com/docs/html/class_m_p_u6050.html
    * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL (Register 27). For each full scale setting,
    * the gyroscopes' sensitivity per LSB in GYRO_xOUT is shown in the table below:
    *
    *    FS_SEL | Full Scale Range   | LSB Sensitivity
    *    -------+--------------------+----------------
    *    0      | +/- 250 degrees/s  | 131  LSB/deg/s
    *    1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
    *    2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
    *    3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
    */
    ITVec3<int16_t> *gyroData;

    /**
     * Raw acceleration readings.
     *
     * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS (Register 28). For each full scale
     * setting, the accelerometers' sensitivity per LSB in ACCEL_xOUT is shown in the table below:
     *
     *    AFS_SEL | Full Scale Range | LSB Sensitivity
     *    --------+------------------+----------------
     *    0       | +/- 2g           | 16384 LSB/g
     *    1       | +/- 4g           | 8192  LSB/g
     *    2       | +/- 8g           | 4096  LSB/g
     *    3       | +/- 16g          | 2048  LSB/g
     */
    ITVec3<int16_t> *accelData;


    // Sensitivity Settings
    short    LSB_PER_G;           // units: LSB/g
    it_float LSB_PER_DEG_PER_SEC; // units: LSB/Ã‚Â°/s
    constexpr static short    ACC_SENSITIVITIES[4] = {16384, 8192, 4096, 2048};
    constexpr static it_float GYR_SENSITIVITIES[4] = {131, 65.5, 32.8, 16.4};
    constexpr static it_float ALPHA = .965; // Alpha value for complimentary filter

    void getAccelerationAndRotation();
    void flipAngle(it_float *angle);
    short getUpOrDown();
};


#endif //CUBENSIS_IMU_H
