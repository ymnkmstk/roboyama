/*
    FilteredColorSensor.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef FilteredColorSensor_hpp
#define FilteredColorSensor_hpp

#include "ColorSensor.h"
#include "FIR.hpp"

class FilteredColorSensor : public ev3api::ColorSensor {
public:
    FilteredColorSensor(ePortS port);
    ~FilteredColorSensor(void);
    void getRawColor(rgb_raw_t &rgb) const;
    void sense();
protected:
    /* FIR filter parameter for normalized cut-off frequency 0.2 by Hamming window function */
    static const int FIR_ORDER = 4; 
    constexpr static const double hn[FIR_ORDER+1] = { 7.483914270309116e-03, 1.634745733863819e-01, 4.000000000000000e-01, 1.634745733863819e-01, 7.483914270309116e-03 };

    FIR_Transposed<FIR_ORDER> *fir_r, *fir_g, *fir_b;
    rgb_raw_t filtered_rgb;
    int fillFIR;
};

#endif /* FilteredColorSensor_hpp */