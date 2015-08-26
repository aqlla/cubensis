//
// Created by Aquilla Sherrock on 8/21/15.
//

#ifndef CUBENSIS_UTIL_H
#define CUBENSIS_UTIL_H

#include "Arduino.h"

#define DBG_NONE            0
#define DBG_READABLE        1
#define DBG_ARSILISCOPE     2
#define CUBENSIS_DBG        DBG_ARSILISCOPE //DBG_READABLE //

#if defined(Arduino_h)
    #if CUBENSIS_DBG==DBG_NONE
        #define CUBE_PRINT(x)   ;
        #define CUBE_PRINTLN(x) ;
        #define PRINT_DELAY     0
    #else
        #define START_SERIAL(x) Serial.begin(x);
        #define CUBE_PRINT(x)   Serial.print(x)
        #define CUBE_PRINTLN(x) Serial.print(x); Serial.print("\n")
    #endif

    #if CUBENSIS_DBG==DBG_ARSILISCOPE
        #define PRINT_DELAY     100
    #elif CUBENSIS_DBG==DBG_READABLE
        #define PRINT_DELAY     450
    #endif
#endif


/* Define THROTTLE_POT if using potentiometer as throttle control */
//#define THROTTLE_POT

#define CH1
#define CH2
#define CH3 11
#define CH4
#define CH5
#define CH6

#define ROLL_PIN     CH1
#define PITCH_PIN    CH2
#define THROT_PIN    CH3
#define YAW_PIN      CH4

#define CH1_PULSEIN_MIN 1090
#define CH1_PULSEIN_MAX 1880
#define CH2_PULSEIN_MIN 1120
#define CH2_PULSEIN_MAX 1900
#define CH3_PULSEIN_MIN 1115
#define CH3_PULSEIN_MAX 2000
#define CH4_PULSEIN_MIN 1115
#define CH4_PULSEIN_MAX 1890
#define CH5_PULSEIN_THRESHOLD 1500
#define CH6_PULSEIN_MIN 1015
#define CH6_PULSEIN_MAX 1980

using cfloat = double;

#endif //CUBENSIS_UTIL_H
