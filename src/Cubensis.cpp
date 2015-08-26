//
// Created by Aquilla Sherrock on 5/29/15.
//

#include "cubensis.h"

Cubensis::Cubensis()
        : status{Status::SLEEP},
          lastPrint{0},
          now{millis()},
          motors{{MOTOR1_PIN},
                 {MOTOR2_PIN},
                 {MOTOR3_PIN},
                 {MOTOR4_PIN}}
{
    Wire.begin();   // use i2cdev fastwire implementation
    TWBR = 24;      // 400kHz I2C clock (200kHz if CPU is 8MHz)

    #if CUBENSIS_DBG!=DBG_NONE
    START_SERIAL(115200);
    CUBE_PRINTLN("Starting Cubensis");
    #endif

    imu1 = new IMU{IMU::Address::LO};
    imu2 = new IMU{IMU::Address::HI};

    // Set kill pin mode to input
    pinMode(CH3, INPUT);

    pinMode(KILL_PIN, INPUT);
    pinMode(STATUS1_LED_PIN, OUTPUT);
    pinMode(STATUS2_LED_PIN, OUTPUT);

    // Verify that all subsystems are working.
    check_status();

    if (status == Status::READY) {
        imu1->setAccelerometerSensitivity(3);
        imu2->setAccelerometerSensitivity(3);
        imu1->setGyroscopeSensitivity(3);
        imu2->setGyroscopeSensitivity(3);

        digitalWrite(STATUS1_LED_PIN, HIGH);
        digitalWrite(STATUS2_LED_PIN, LOW);

        // Initialize PID
        pid_stab.x = new PID{&orientation.x,  &error_stab.x, &setpoint_stab.x, 3, 0, .2};
        pid_rate.x = new PID{&rotationRate.x, &error_rate.x, &error_stab.x, .5, .000025, .0075};

        #if CUBENSIS_DBG!=DBG_NONE
        CUBE_PRINTLN("Cubensis Started successfully\n");
        #endif
    }
}

auto Cubensis::check_status() -> Status {
    if (imu1->ok() && imu2->ok()) {
        status = Status::READY;
    } else {
        status = Status::ERROR;

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

    return status;
}

/**
 * Prepare Quadrotor for Flight.
 * Attach all motor speed controllers to the correct pin and write the correct
 * startup value to turn them on.
 */
void Cubensis::start_motors()
{
#if CUBENSIS_DBG==DBG_READABLE
    CUBE_PRINTLN("Initializing Motors.");
#endif

    for (int8_t i = 0; i < MOTOR_COUNT; ++i)
        motors[i].init();

    status = Status::RUNNING;

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
void Cubensis::calibrate_sensors(unsigned long time)
{
    #if CUBENSIS_DBG==DBG_READABLE
    CUBE_PRINTLN("Calibrating Sensors.");
    #endif

    imu1->calibrate(time);
    imu2->calibrate(time);

    digitalWrite(STATUS2_LED_PIN, HIGH);

    #if CUBENSIS_DBG==DBG_READABLE
    CUBE_PRINTLN("Calibration Finished.");
    CUBE_PRINTLN("Offsets:");
    imu1->accelOffset.print();
    imu2->accelOffset.print();
    imu1->gyroOffset.print();
    imu2->gyroOffset.print();
    #endif
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
    if (USE_KILL_SWITCH && digitalRead(KILL_PIN) == KILL_SIGNAL)
        kill();

    imu1->updateOrientation();
    imu2->updateOrientation();

    orientation  = (imu1->complementary + imu2->complementary) / 2;
    rotationRate = (imu1->rotation + imu2->rotation) / 2;
    pid_stab.x->computeError();
    pid_rate.x->computeError();

    Motor::read_throttle_pin();

    motors[0].set();
    motors[1].set_error(error_rate.x);
    motors[2].set();
    motors[3].set_error(-error_rate.x);

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
    CUBE_PRINTLN("received kill signal.");

    if (!Motor::killed) {
        status = Status::KILL;
        Motor::kill(true);
        digitalWrite(STATUS2_LED_PIN, LOW);
    } else {
        status = Status::RUNNING;
        Motor::kill(false);
        digitalWrite(STATUS2_LED_PIN, HIGH);
    }
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
        CUBE_PRINT(error_rate.x);
        CUBE_PRINT(",\"yaw\": ");
        CUBE_PRINT(error_stab.x);
        CUBE_PRINT(",\"motor1\": ");
        CUBE_PRINT(motors[0].throttle);
        CUBE_PRINT(",\"motor2\": ");
        CUBE_PRINT(motors[1].throttle);
        CUBE_PRINT(",\"motor3\": ");
        CUBE_PRINT(motors[2].throttle);
        CUBE_PRINT(",\"motor4\": ");
        CUBE_PRINT(motors[3].throttle);
        CUBE_PRINTLN("}");
        #elif CUBENSIS_DBG==DBG_READABLE
        CUBE_PRINT("Throttle: ");
        CUBE_PRINTLN(pulseIn(CH3, HIGH, 25000));
//        CUBE_PRINT("Rate error: ");
//        CUBE_PRINTLN(error_rate.x);
//        CUBE_PRINT("Stab error: ");
//        CUBE_PRINTLN(error_stab.x);
        #endif
    }
}