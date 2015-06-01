
/* ************************************************************************** *
 * Cubensis Control File													  *
 * Aquilla Sherrock - 16 May 2015											  *
 * ************************************************************************** *
 */


#include <Arduino.h>
#include <I2Cdev.h>
#include "Cubensis.h"


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
    Wire.begin();
    TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
    Serial.begin(115200);

    PRINTLN("Starting devices.");
    cube = new Cubensis();

    if (cube->status != CUBENSIS_STATUS_RUNNING) {
        PRINT("Error starting devices.");
        while(1) delay(100);
    }


    cube->startMotors1();
    cube->calibrate(8000);
    cube->startMotors2();
    cube->start();
    lastPrint = millis();
}


void loop() {
    cube->update();

//    unsigned long now = millis();
//    if (now - lastPrint > PRINT_DELAY) {
//        #ifdef ARSILISCOPE
//        cube->arsiliscope();
//        #endif
//
//        lastPrint = now;
//    }
}



