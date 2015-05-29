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

#define ADDR_AD0_LOW  MPU6050_ADDRESS_AD0_LOW
#define ADDR_AD0_HIGH MPU6050_ADDRESS_AD0_HIGH


#if defined(Arduino_h)
    #ifdef TIME_IN_MICROS
        #define TIME_FUNC micros
        #define TIME_TO_SEC 1000000.0
    #else
        #define TIME_FUNC millis
        #define TIME_TO_SEC 1000.0
    #endif
#endif


class IMU
{
public:
    short status;
    short address;

    ITVec3<it_float>* complementaryAngle;
    ITVec3<it_float>* orientation;
    ITVec3<it_float>* rotation;
    AccelerationVec*  acceleration;

    IMU(short address): status(IMU_STATUS_SLEEP)
    {
        if (address != ADDR_AD0_LOW && address != ADDR_AD0_HIGH) {
            status = IMU_STATUS_ADDRESS_ERROR;
        } else {
            device = new MPU6050();

            rotation = new ITVec3<it_float>();
            acceleration = new AccelerationVec();
            orientation  = new ITVec3<it_float>();
            complementaryAngle = new ITVec3<it_float>();

            gyroOffset  = new ITVec3<it_float>();
            accelOffset = new ITVec3<it_float>();

            gyroData  = new ITVec3<int16_t>();
            accelData = new ITVec3<int16_t>();
        }
    };

    ~IMU()
    {
        delete device;
        delete rotation;
        delete gyroData;
        delete accelData;
        delete gyroOffset;
        delete accelOffset;
        delete orientation;
        delete acceleration;
        delete complementaryAngle;
    };


    /**
     * Initialize.
     * 
     * Attempt to wake IMU and check if a successful connection to the device
     * could be made.
     *
     * @return true if device runs properly, else false.
     */
    bool init()
    {
        device->initialize();
        if (device->testConnection()) {
            status = IMU_STATUS_OK;
            setGyroscopeSensitivity(0);
            setAccelerometerSensitivity(0);
            return true;
        } else {
            status = IMU_STATUS_CONNECTION_ERROR;
            return false;
        }
    };


    /**
     * Update Orientation.
     * Get the values from the IMU and update the current axial orientation
     * values based on gyroscope readings, calculates the roll and pitch from
     * accelerometer data, and combines these values with a complementary
     * filter.
     */
    void updateOrientation()
    {
        getAccelerationAndRotation();

        short zDirection = getUpOrDown();
        orientation->x += rotation->x * zDirection;
        orientation->y += rotation->y * zDirection;
        orientation->z += rotation->z;

        flipAngle(&orientation->x);
        flipAngle(&orientation->y);
        flipAngle(&orientation->z);

        complementaryAngle->x = ALPHA * orientation->x + (1 - ALPHA) * acceleration->roll();
        complementaryAngle->y = ALPHA * orientation->y + (1 - ALPHA) * acceleration->pitch();
        complementaryAngle->z = orientation->z;
    };

    /**
     * Calibrate the IMU device.
     *
     * Measures the average accelerometer and gyroscope data over a given
     * number of iterations (preferably when the device is steady and the
     * z-azis is normal to gravity) and accordingly sets offset values per axis.
     *
     * @param iterations: number of times to scan IMU readings.
     */
    void calibrate(short iterations)
    {
        for (short i = 0; i < iterations; ++i) {
            device->getMotion6(&accelData->x, &accelData->y, &accelData->z, &gyroData->x, &gyroData->y, &gyroData->z);

            // Convert acceleration LSB/mg values to mg
            accelOffset->x += ((it_float) accelData->x / LSB_PER_G);
            accelOffset->y += ((it_float) accelData->y / LSB_PER_G);
            accelOffset->z += ((it_float) accelData->z / LSB_PER_G);

            // Convert gyroscope LSB/deg/s values to deg/sec
            gyroOffset->x += ((it_float) gyroData->x / LSB_PER_DEG_PER_SEC);
            gyroOffset->y += ((it_float) gyroData->y / LSB_PER_DEG_PER_SEC);
            gyroOffset->z += ((it_float) gyroData->z / LSB_PER_DEG_PER_SEC);
        }

        // calculate averages
        accelOffset->x /= (it_float) iterations;
        accelOffset->y /= (it_float) iterations;
        accelOffset->z /= (it_float) iterations;
        gyroOffset->x  /= (it_float) iterations;
        gyroOffset->y  /= (it_float) iterations;
        gyroOffset->z  /= (it_float) iterations;

        // Account for gravity
        accelOffset->z -= 1;
    };


    /**
     * Set gyroscope sensitivity.
     */
    void setGyroscopeSensitivity(uint8_t sensitivity)
    {
        if (sensitivity < 0 || sensitivity >= 4) {
            sensitivity = 0;
        }

        device->setFullScaleGyroRange(sensitivity);
        LSB_PER_DEG_PER_SEC = GYR_SENSITIVITIES[sensitivity];
    };


    /**
     * Set accelerometer sensitivity.
     */
    void setAccelerometerSensitivity(uint8_t sensitivity)
    {
        if (sensitivity < 0 || sensitivity >= 4) {
            sensitivity = 0;
        }

        device->setFullScaleAccelRange(sensitivity);
        LSB_PER_G = ACC_SENSITIVITIES[sensitivity];
    };


    /**
     * Initialize the previous time value;
     */
    void startTime() {
        previousTime = TIME_FUNC();
    };

private:
    // The IMU device 
    MPU6050* device;
    unsigned long previousTime;
    ITVec3<it_float>* gyroOffset;
    ITVec3<it_float>* accelOffset;

    // Sensitivity Settings
    short    LSB_PER_G;           // units: LSB/g
    it_float LSB_PER_DEG_PER_SEC; // units: LSB/Ã‚Â°/s
    constexpr static short    ACC_SENSITIVITIES[4] = {16384, 8192, 4096, 2048};
    constexpr static it_float GYR_SENSITIVITIES[4] = {131, 65.5, 32.8, 16.4};

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
    ITVec3<int16_t>* gyroData;

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
    ITVec3<int16_t>* accelData;


    /** 
     * Read in new IMU values.
     *
     * Get raw acceleration and gyroscopic rate data from IMU and convert to
     * mg & deg/s values respectively.
     */
    void getAccelerationAndRotation()
    {
        device->getMotion6(&accelData->x, &accelData->y, &accelData->z, &gyroData->x, &gyroData->y, &gyroData->z);

        unsigned long currentTime = TIME_FUNC();
        it_float dt = (currentTime - previousTime) / TIME_TO_SEC;
        previousTime = currentTime;

        this->rotation->x = 0 - ((it_float) gyroData->x / LSB_PER_DEG_PER_SEC - gyroOffset->x) * dt;
        this->rotation->y = 0 - ((it_float) gyroData->y / LSB_PER_DEG_PER_SEC - gyroOffset->y) * dt;
        this->rotation->z = 0 - ((it_float) gyroData->z / LSB_PER_DEG_PER_SEC - gyroOffset->z) * dt;

        acceleration->x = ((it_float) accelData->x / LSB_PER_G) - accelOffset->x * 9.81;
        acceleration->y = ((it_float) accelData->y / LSB_PER_G) - accelOffset->y * 9.81;
        acceleration->z = ((it_float) accelData->z / LSB_PER_G) - accelOffset->z * 9.81;
    };


    it_float flipAngle(it_float *angle)
    {
        if (*angle > 180) {
            *angle = -180 - (*angle - 180);
        } else if (*angle < -180) {
            *angle = 180 + (*angle + 180);
        }

        return *angle;
    };


    short getUpOrDown()
    {
        return acceleration->z > 0
                ? (short) 1
                : (short) -1;
    };
};


#endif //CUBENSIS_IMU_H