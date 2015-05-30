
/* ************************************************************************** *
 * Cubensis Control File													  *
 * Aquilla Sherrock - 16 May 2015											  *
 * ************************************************************************** *
 */

#include <Arduino.h>
#include "Cubensis.h"

Cubensis* cube;

void setup() {
    cube = new Cubensis();
    cube->startMotors();
    cube->calibrate(10000);
    cube->start();
}

void loop() {
    cube->update();
}



