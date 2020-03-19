/**
 * \file: cal_temp.cpp
 *
 * Calibration helper class
 *
 * Copyright (C) 2015 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#include <Arduino.h>

#include <stdio.h>

#include "cal_temp.h"

// set parameters to default zero bias and 1.0 scaling factor
void CalTemp::defaults()
{
    min_temp = 27.0;
    max_temp = 27.0;
    coeffs[0] = 0.0;
    coeffs[1] = 0.0;
    coeffs[2] = 0.0;
}


CalTemp::CalTemp()
{
    defaults();
}


CalTemp::~CalTemp()
{
    // nothing to do
}


// load parameters from specified property subtree
void CalTemp::init( float calib[3], float min_temp, float max_temp )
{
    min_temp = min_temp;
    max_temp = max_temp;
    for ( int i = 0; i < 3; i++ ) {
        coeffs[i] = calib[i];
    }
}


