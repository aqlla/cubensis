//
// Created by Tyler Cook on 5/31/15.
//

#ifndef PROJECT_NAME_BASESTATION_H
#define PROJECT_NAME_BASESTATION_H


#include <Arduino.h>
#include <Wire.h>

class BaseStation {
private:
    const int DATALEN = 24;
    const int ADDRESS = 8;
    union MyFloat{
        byte b[4];
        float val;
    } ;

    void parseFloats(float* out, byte* in) {
        MyFloat mf;
        int c1 = 0;
        int c2 = 0;
        int c3 = 0;

        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        out[c3++] = mf.val;

        c1 = 0;
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        out[c3++] = mf.val;

        c1 = 0;
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        out[c3++] = mf.val;

        c1 = 0;
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        out[c3++] = mf.val;

        c1 = 0;
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        out[c3++] = mf.val;
        
        c1 = 0;
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        mf.b[c1++] = in[c2++];
        out[c3++] = mf.val;
    }
public:
    void updatePid(void (*f)(float, float, float, float, float, float)) {
        bool dataComplete = false;
        int count = 0;
        byte data[DATALEN];
        for (int i = 0; i < DATALEN; ++i) {
            data[i] = 0x00;
        }
        float floatData[DATALEN];
        while (!dataComplete) {
            Wire.requestFrom(ADDRESS, DATALEN);
            count = 0;
            while (Wire.available()) {
                data[count++] = Wire.read();
            }
            for (int i = 0; i < count; i++) {
                if (data[i] != 0xFF) {
                    dataComplete = true;
                    break;
                }
            }
            delay(100);
        }
        parseFloats(floatData, data);
        f(floatData[0], floatData[1], floatData[2], floatData[3], floatData[4], floatData[5]);
    }
};


#endif //PROJECT_NAME_BASESTATION_H
