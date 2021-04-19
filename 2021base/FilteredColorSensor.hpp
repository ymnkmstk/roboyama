/*
    FilteredColorSensor.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef FilteredColorSensor_hpp
#define FilteredColorSensor_hpp

#include "ColorSensor.h"
#include "FIR.hpp"
/* FIR filter parameter */
static const int FIR_ORDER = 10; 
static const double hn[FIR_ORDER+1] = { -1.247414986406201e-18, -1.270350182429102e-02, -2.481243022283666e-02, 6.381419731491805e-02, 2.761351394755998e-01, 4.000000000000000e-01, 2.761351394755998e-01, 6.381419731491805e-02, -2.481243022283666e-02, -1.270350182429102e-02, -1.247414986406201e-18 };

class FilteredColorSensor : public ev3api::ColorSensor {
public:
    FilteredColorSensor(ePortS port);
    ~FilteredColorSensor(void);
    void getRawColor(rgb_raw_t &rgb) const;
    void sense();
protected:
    FIR_Transposed<FIR_ORDER> *fir_r, *fir_g, *fir_b;
    rgb_raw_t filtered_rgb;
    int fillFIR;
};

#endif /* FilteredColorSensor_hpp */