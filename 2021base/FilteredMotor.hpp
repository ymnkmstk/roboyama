/*
    FilteredMotor.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef FilteredMotor_hpp
#define FilteredMotor_hpp

#include "Motor.h"
#include "FIR.hpp"

#ifndef FIR_PARM
#define FIR_PARM
/* FIR filter parameter */
static const int FIR_ORDER = 10; 
static const double hn[FIR_ORDER+1] = { -1.247414986406201e-18, -1.270350182429102e-02, -2.481243022283666e-02, 6.381419731491805e-02, 2.761351394755998e-01, 4.000000000000000e-01, 2.761351394755998e-01, 6.381419731491805e-02, -2.481243022283666e-02, -1.270350182429102e-02, -1.247414986406201e-18 };
#endif /* FIR_PARM */

class FilteredMotor : public ev3api::Motor {
public:
    FilteredMotor(ePortM port);
    ~FilteredMotor(void);
    void setPWM(int pwm);
protected:
    FIR_Transposed<FIR_ORDER> *fir_pwm;
    int fillFIR, filtered_pwm;
};

#endif /* FilteredMotor_hpp */