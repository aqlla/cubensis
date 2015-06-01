// Created by Aquilla Sherrock on 5/31/15.
// Copyright (c) 2015 Insignificant Tech. All rights reserved.


#ifndef CUBENSIS_BS2_H
#define CUBENSIS_BS2_H

#include "itvec.h"

union bytef {
    it_float value;
    struct bytes {
        unsigned long one   :4;
        unsigned long two   :4;
        unsigned long three :4;
        unsigned long four  :4;
    };
};


void bytesToFloat()

#endif //CUBENSIS_BS2_H
