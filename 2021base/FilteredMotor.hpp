/*
    FilteredMotor.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef FilteredMotor_hpp
#define FilteredMotor_hpp

#include "Motor.h"
#include "FIR.hpp"

class FilteredMotor : public ev3api::Motor {
public:
    FilteredMotor(ePortM port);
    ~FilteredMotor(void);
    void setPWM(int pwm);
protected:
    /* FIR filter parameter for normalized cut-off frequency 0.49 */
    static const int FIR_ORDER = 4; 
    constexpr static const double hn[FIR_ORDER+1] = { -1.595792292436011e-03, 1.079289528739032e-02, 9.800000000000000e-01, 1.079289528739032e-02, -1.595792292436011e-03 };

    FIR_Transposed<FIR_ORDER> *fir_pwm;
    int fillFIR, filtered_pwm;
};

#endif /* FilteredMotor_hpp */