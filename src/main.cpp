
/* ************************************************************************** *
 * Cubensis Control File													  *
 * Aquilla Sherrock - 16 May 2015											  *
 * ************************************************************************** *
 */


#include <Arduino.h>
#include <I2Cdev.h>
#include "Cubensis.h"

#define DEBUG
#define ARSILISCOPE

#ifdef ARSILISCOPE
    #define PRINT(x)    ;
    #define PRINTLN(x)  ;
    #define PRINT_DELAY 50
#else
    #define PRINT(x)    Serial.print(x)
    #define PRINTLN(x)  Serial.print(String(x) + "\n")
    #define PRINT_DELAY 200
#endif

Cubensis* cube;
unsigned long lastPrint = 0;

void setup() {
    Wire.begin();   // use i2cdev fastwire implementation
    TWBR = 24;      // 400kHz I2C clock (200kHz if CPU is 8MHz)
    Serial.begin(115200);
    cube = new Cubensis();

    PRINTLN("Starting Cubensis.");

    if (cube->status != CUBENSIS_STATUS_RUNNING) {
        PRINT("Error starting devices.\nCubrnsis status: ");
        PRINTLN(cube->status);
        return;
    }


    cube->startMotors();
    cube->calibrate(3000);
    delay(1000);
    cube->start();
    lastPrint = millis();
}


void loop() {
    cube->update();

#ifdef DEBUG
    unsigned long now = millis();
    if (now - lastPrint > PRINT_DELAY) {
        cube->print();
        lastPrint = now;
    }
#endif
}



