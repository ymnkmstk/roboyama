/*
    FilteredMotor.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef FilteredMotor_hpp
#define FilteredMotor_hpp

#include "Motor.h"
#include "FIR.hpp"

/* FIR filter parameter */
static const int MFIR_ORDER = 10; 
static const double mhn[MFIR_ORDER+1] = { 1.573810628933549e-03, -3.321813563286734e-03, 7.910007566689100e-03, -1.360707786661095e-02, 1.823095533324903e-02, 9.800000000000000e-01, 1.823095533324903e-02, -1.360707786661095e-02, 7.910007566689100e-03, -3.321813563286734e-03, 1.573810628933549e-03 };

class FilteredMotor : public ev3api::Motor {
public:
    FilteredMotor(ePortM port);
    ~FilteredMotor(void);
    void setPWM(int pwm);
protected:
    FIR_Transposed<MFIR_ORDER> *fir_pwm;
    int fillFIR, filtered_pwm;
};

#endif /* FilteredMotor_hpp */