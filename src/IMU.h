//
// Created by Aquilla Sherrock on 5/16/15.
//

#ifndef CUBENSIS_IMU_H
#define CUBENSIS_IMU_H

#include "util.h"
#include "MPU6050.h"
#include "itvec.h"

#define TIME_FUNC millis
#define TIME_TO_SEC 1000.0


class IMU
{
public:
    enum class Status {
        SLEEP, OK, CONNECTION_ERROR, ADDRESS_ERROR
    };

    enum class Address {
        LO = MPU6050_ADDRESS_AD0_LOW,
        HI = MPU6050_ADDRESS_AD0_HIGH
    };

    it::vec3<cfloat> *gyroOffset;
    it::vec3<cfloat> *accelOffset;

    it::vec3<cfloat> *rotation;
    it::vec3<cfloat> *complementary;

    IMU(Address address);
    ~IMU();

    void updateOrientation();
    void calibrate(unsigned long);
    void setGyroscopeSensitivity(uint8_t);
    void setAccelerometerSensitivity(uint8_t);
    void startTime();
    bool ok();

private:
    // The IMU device 
    MPU6050* device;
    Status status;
    Address address;
    unsigned long previousTime;

    it::vec3<cfloat> *orientation;
    it::AccelerationVec  *acceleration;

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
    it::vec3<int16_t> *gyroData;

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
    it::vec3<int16_t> *accelData;


    // Sensitivity Settings
    uint16_t LSB_PER_G;         // units: LSB/g
    cfloat LSB_PER_DEG_PER_SEC; // units: LSB/Ã‚Â°/s
    static uint16_t ACC_SENSITIVITIES[4];
    static cfloat   GYR_SENSITIVITIES[4];
    constexpr static cfloat ALPHA = .9995; // Alpha value for complimentary filter

    void getAccelerationAndRotation();
    void flipAngle(cfloat*angle);
    short getUpOrDown();
};


#endif //CUBENSIS_IMU_H
