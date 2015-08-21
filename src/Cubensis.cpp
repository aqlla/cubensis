//
// Created by Aquilla Sherrock on 5/29/15.
//

#include "Cubensis.h"

unsigned long Cubensis::lastPrint = millis();
unsigned long Cubensis::now = 0;

Cubensis::Cubensis()
        : status{CubensisStatus::SLEEP},
        motor1{MOTOR1_PIN},
        motor2{MOTOR2_PIN},
        motor3{MOTOR3_PIN},
        motor4{MOTOR4_PIN}
{
    Wire.begin();   // use i2cdev fastwire implementation
    TWBR = 24;      // 400kHz I2C clock (200kHz if CPU is 8MHz)

    #if CUBENSIS_DBG!=DBG_NONE
    START_SERIAL(115200);
    CUBE_PRINTLN("Starting Cubensis");
    #endif


    // Initialize IMU sensors.
    imu1 = new IMU{IMU_ADDR_AD0_LOW};
    imu2 = new IMU{IMU_ADDR_AD0_HIGH};

    // Set kill pin mode to input
    pinMode(KILL_PIN, INPUT);


    // Verify that all subsystems are working.
    if (check_status() == CubensisStatus::READY) {
        // Initialize PID
        error_stabx = 0;
        error_ratex = 0;
        setpoint_stabx = 0;
        pid_stabx = new PID{&orientation.x,  &error_stabx, &setpoint_stabx, 1, 0, 0};
        pid_ratex = new PID{&rotationRate.x, &error_ratex, &error_stabx, 0.375, 0, 0};

        #if CUBENSIS_DBG!=DBG_NONE
        CUBE_PRINTLN("Cubensis Started successfully\n");
        #endif
    }
}

auto Cubensis::check_status() -> CubensisStatus {
    if (imu1->ok() && imu2->ok()) {
        status = CubensisStatus::READY;
    } else {
        status = CubensisStatus::ERROR;

        if (!imu1->ok()) {
            #if CUBENSIS_DBG!=DBG_NONE
            CUBE_PRINTLN("ERROR: IMU1.");
            #endif
        } else if (!imu2->ok()) {
            #if CUBENSIS_DBG!=DBG_NONE
            CUBE_PRINTLN("ERROR: IMU2.");
            #endif
        }
    }
}

/**
 * Prepare Quadrotor for Flight.
 * Attach all motor speed controllers to the correct pin and write the correct
 * startup value to turn them on.
 */
void Cubensis::prepare()
{
    motor1.init();
    motor2.init();
    motor3.init();
    motor4.init();

    status = CubensisStatus::RUNNING;

    #if CUBENSIS_DBG!=DBG_NONE
    print();
    #endif
}


/**
 * Calibrate Sensors.
 *
 * Allows sensors to adjust to noise and drift by reading their values while
 * the quadrotor is in a stable and level state. Sensors will set offsets
 * reflecting the erroneous data they read during the requested time period.
 *
 * @param calibrationTime: amount of time to calibrate the sensors.
 */
void Cubensis::calibrateSensors(unsigned long calibrationTime)
{
    imu1->calibrate(calibrationTime);
    imu2->calibrate(calibrationTime);
}


/**
 * Start Quadrotor.
 *
 * Initiates normal quadrotoring operation. Starts the sensors' previous time
 * value so the first elapsed time measurement will be closer to accurate.
 */
void Cubensis::start()
{
    imu1->startTime();
    imu2->startTime();
    update();
}


/**
 * Update State.
 *
 * Checks the kill pin to verify that procession may be done safely. If not,
 * the kill method is called, halting normal operation. If the switch is not
 * set to kill, the current estimation of the quadrotor's orientation state is
 * updated, and appropriate reactions are made. Calls update methods on all
 * sensors, computes the new error value from the PID controllers, and then
 * sets the throttle values of the motors accordingly.
 */
void Cubensis::update()
{
    if (status != CubensisStatus::KILL) {
        if (USE_KILL_SWITCH && digitalRead(KILL_PIN) == KILL_SIGNAL)
            return kill();

        imu1->updateOrientation();
        imu2->updateOrientation();

        orientation  = (*imu1->complementary + *imu2->complementary) / 2;
        rotationRate = (*imu1->rotation + *imu2->rotation) / 2;
        pid_stabx->computeError();
        pid_ratex->computeError();

        Motor::getThrottlePinValue();
        motor1.set(0);
        motor2.set(error_ratex);
        motor3.set(0);
        motor4.set(-error_ratex);
    } else if (digitalRead(KILL_PIN) != KILL_SIGNAL) {
        // TODO: un kill
        //Motor::kill(false);
        status = CubensisStatus::RUNNING;
    }

    #if CUBENSIS_DBG!=DBG_NONE
    print();
    #endif
}


/**
 * Halt and Catch Fire.
 *
 * Stops normal quadrotoring operations to prevent loss of blood. Mostly just
 * attempts to stop the propellors from spinning at deadly RPMs.
 */
void Cubensis::kill() {
    status = CubensisStatus::KILL;
    motor1.kill();
    motor2.kill();
    motor3.kill();
    motor4.kill();
}


/**
 * Print.
 * Prints state information. Duh.
 */
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