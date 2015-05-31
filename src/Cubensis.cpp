//
// Created by Aquilla Sherrock on 5/29/15.
//

#include "Cubensis.h"

unsigned long Cubensis::lastPrint = millis();
unsigned long Cubensis::now = 0;

Cubensis::Cubensis()
    :motor1(MOTOR1_PIN),
     motor2(MOTOR2_PIN),
     motor3(MOTOR3_PIN),
     motor4(MOTOR4_PIN)
{
    status = CUBENSIS_STATUS_SLEEP;
    imu1 = new IMU(IMU1_ADDR);
    imu2 = new IMU(IMU2_ADDR);
    bool imu1Status = imu1->init();
    bool imu2Status = imu2->init();

    Wire.begin();   // use i2cdev fastwire implementation
    TWBR = 24;      // 400kHz I2C clock (200kHz if CPU is 8MHz)

    #if CUBENSIS_DBG!=DBG_NONE
    START_SERIAL(115200);
    CUBE_PRINTLN("Starting Cubensis");
    #endif

    if (imu1Status != IMU_STATUS_OK || imu2Status != IMU_STATUS_OK) {
        status = CUBENSIS_STATUS_ERROR_BOTH_IMU;
        #if CUBENSIS_DBG!=DBG_NONE
        CUBE_PRINTLN("ERROR: BOTH IMUS.");
        #endif

        if (imu1Status == IMU_STATUS_OK) {
            status = CUBENSIS_STATUS_ERROR_IMU2;
            #if CUBENSIS_DBG!=DBG_NONE
            CUBE_PRINTLN("ERROR: IMU2.");
            #endif
        } else if (imu2Status == IMU_STATUS_OK) {
            status = CUBENSIS_STATUS_ERROR_IMU1;
            #if CUBENSIS_DBG!=DBG_NONE
            CUBE_PRINTLN("ERROR: IMU1.");
            #endif
        }
    } else {
        error_stabx = 0;
        error_ratex = 0;
        setpoint_stabx = 0;
        pid_stabx = new PID(&orientation.x,  &error_stabx, &setpoint_stabx, 1, 0, 0);
        pid_ratex = new PID(&rotationRate.x, &error_ratex, &error_stabx, 0.375, 0, 0);

        pinMode(KILL_PIN, INPUT);
        status = CUBENSIS_STATUS_RUNNING;
        #if CUBENSIS_DBG!=DBG_NONE
        CUBE_PRINTLN("Cubensis Started successfully\n");
        #endif
    }
}


void Cubensis::startMotors()
{
    motor1.init();
    motor2.init();
    motor3.init();
    motor4.init();

    #if CUBENSIS_DBG!=DBG_NONE
    print();
    #endif
}

void Cubensis::calibrate(unsigned long timeToCalibrate)
{
    imu1->calibrate(timeToCalibrate);
    imu2->calibrate(timeToCalibrate);
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
        if (USE_KILL_SWITCH && digitalRead(KILL_PIN) == KILL_SIGNAL)
            return kill();

        imu1->updateOrientation();
        imu2->updateOrientation();

        orientation  = (*imu1->complementary + *imu2->complementary) / 2;
        rotationRate = (*imu1->rotation + *imu2->rotation) / 2;
        pid_stabx->computeError();
        pid_ratex->computeError();

        Motor::getThrottlePinValue();
        throttle = Motor::throttlePinValue;

        motor1.set(throttle);
        motor2.set(throttle + error_ratex);
        motor3.set(throttle);
        motor4.set(throttle - error_ratex);
    } else if (digitalRead(KILL_PIN) != KILL_SIGNAL) {
        // TODO: un kill
        //Motor::kill(false);
        status = CUBENSIS_STATUS_RUNNING;
    }

    #if CUBENSIS_DBG!=DBG_NONE
    print();
    #endif
}


void Cubensis::kill() {
    status = CUBENSIS_STATUS_KILL;
    motor1.kill();
    motor2.kill();
    motor3.kill();
    motor4.kill();
}


void Cubensis::print()
{
    now = millis();
    if (now - lastPrint > PRINT_DELAY) {
        lastPrint = now;

        #if CUBENSIS_DBG==DBG_ARSILISCOPE
        CUBE_PRINT("{\"roll\": ");
        CUBE_PRINT(orientation.x);
        CUBE_PRINT(",\"pitch\": ");
        CUBE_PRINT(error_ratex);
        CUBE_PRINT(",\"yaw\": ");
        CUBE_PRINT(error_stabx);
        CUBE_PRINT(",\"motor1\": ");
        CUBE_PRINT(motor1.getThrottle());
        CUBE_PRINT(",\"motor2\": ");
        CUBE_PRINT(motor2.getThrottle());
        CUBE_PRINT(",\"motor3\": ");
        CUBE_PRINT(motor3.getThrottle());
        CUBE_PRINT(",\"motor4\": ");
        CUBE_PRINT(motor4.getThrottle());
        CUBE_PRINTLN("}");
        #endif
    }
}