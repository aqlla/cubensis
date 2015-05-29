include_directories(../lib/core)
include_directories(../lib/i2cdev)
include_directories(../lib/mpu6050)
include_directories(../lib/servo)

link_directories(../lib/core)
link_directories(../lib/i2cdev)
link_directories(../lib/mpu6050)
link_directories(../lib/servo)

link_libraries(ARDUINO_CORE)
link_libraries(I2CDEV)
link_libraries(MPU6050)

add_executable(cubensis main.cpp PID.h itvec.h IMU.h Cubensis.h motor.h)