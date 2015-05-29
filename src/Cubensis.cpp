//
// Created by Aquilla Sherrock on 5/29/15.
//

#include "Cubensis.h"

unsigned long Cubensis::lastPrint = millis();
unsigned long Cubensis::now = 0;

Cubensis::Cubensis()
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
        setpoint_ratex = 0; //&error_stabx;
        pid_stabx = new PID(&orientation.x,  &error_stabx, &setpoint_stabx, 0,   0, 0);
        pid_ratex = new PID(&rotationRate.x, &error_ratex, &setpoint_ratex, 15, 100, 1);

        motor1 = new Motor(MOTOR1_PIN);
        motor2 = new Motor(MOTOR2_PIN);
        motor3 = new Motor(MOTOR3_PIN);
        motor4 = new Motor(MOTOR4_PIN);
        pinMode(KILL_PIN, INPUT);

        status = CUBENSIS_STATUS_RUNNING;
        #if CUBENSIS_DBG!=DBG_NONE
        CUBE_PRINTLN("Cubensis Started successfully");
        #endif
    }
}


void Cubensis::startMotors()
{
    motor1->start();
    motor2->start();
    motor3->start();
    motor4->start();
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
        if (USE_KILL_SWITCH && digitalRead(KILL_PIN) == KILL_SIGNAL) {
            status = CUBENSIS_STATUS_KILL;
            Motor::kill();
            return;
        }

        imu1->updateOrientation();
        imu2->updateOrientation();

        orientation  = (*imu1->complementary + *imu2->complementary) / 2;
        rotationRate = (*imu1->rotation + *imu2->rotation) / 2;

        pid_ratex->compute();
        Motor::setThrottle();
        motor2->setError(error_ratex);
        motor4->setError(-error_ratex);

    } else if (digitalRead(KILL_PIN) != KILL_SIGNAL) {
        Motor::kill(false);
        status = CUBENSIS_STATUS_RUNNING;
    }

    #if CUBENSIS_DBG!=DBG_NONE
    print();
    #endif
}


void Cubensis::print()
{
    now = millis();
    if (now - lastPrint > PRINT_DELAY) {
        lastPrint = now;

        #if CUBENSIS_DBG==DBG_ARSILISCOPE
        CUBE_PRINT("{\"roll\": ");
        CUBE_PRINT(rotationRate.x);
        CUBE_PRINT(",\"pitch\": ");
        CUBE_PRINT(error_ratex);
        CUBE_PRINT(",\"yaw\": ");
        CUBE_PRINT(orientation.x);
        CUBE_PRINT(",\"motor1\": ");
        CUBE_PRINT(motor1->getThrottle());
        CUBE_PRINT(",\"motor2\": ");
        CUBE_PRINT(motor2->getThrottle());
        CUBE_PRINT(",\"motor3\": ");
        CUBE_PRINT(motor3->getThrottle());
        CUBE_PRINT(",\"motor4\": ");
        CUBE_PRINT(motor4->getThrottle());
        CUBE_PRINTLN("}");
        #endif
    }
}