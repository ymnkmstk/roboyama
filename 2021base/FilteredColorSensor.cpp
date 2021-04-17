/*
    FilteredColorSensor.cpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#include "FilteredColorSensor.hpp"

FilteredColorSensor::FilteredColorSensor(ePortS port) : ColorSensor(port), fillFIR(FIR_ORDER + 1) {
    fir_r = new FIR_Transposed<FIR_ORDER>(hn);
    fir_g = new FIR_Transposed<FIR_ORDER>(hn);
    fir_b = new FIR_Transposed<FIR_ORDER>(hn);
}

FilteredColorSensor::~FilteredColorSensor() {
    delete fir_b;
    delete fir_g;
    delete fir_r;
}

void FilteredColorSensor::getRawColor(rgb_raw_t &rgb) const {
    /* wait until FIR array is filled */
    if (fillFIR > 0) {
        rgb.r = rgb.g = rgb.b = 0;
    } else {
        rgb.r = filtered_rgb.r;
        rgb.g = filtered_rgb.g;
        rgb.b = filtered_rgb.b;
    }
}

void FilteredColorSensor::sense() {
    ev3api::ColorSensor::getRawColor(filtered_rgb);
    /* process RGB by the Low Pass Filter */
    filtered_rgb.r = fir_r->Execute(filtered_rgb.r);
    filtered_rgb.g = fir_g->Execute(filtered_rgb.g);
    filtered_rgb.b = fir_b->Execute(filtered_rgb.b);
    /* decrement counter */
    fillFIR--;
}