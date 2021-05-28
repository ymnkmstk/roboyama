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
    int32_t getPwm();
protected:
    /* FIR filter parameter for normalized cut-off frequency 0.2 */
    static const int FIR_ORDER = 2; 
    constexpr static const double hn[FIR_ORDER+1] = { 3.027306914562628e-01, 4.000000000000000e-01, 3.027306914562628e-01 };

    FIR_Transposed<FIR_ORDER> *fir_pwm;
    int fillFIR, filtered_pwm;
};

#endif /* FilteredMotor_hpp */