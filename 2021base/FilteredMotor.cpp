/*
    FilteredMotor.cpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#include "FilteredMotor.hpp"

FilteredMotor::FilteredMotor(ePortM port) : Motor(port), fillFIR(MFIR_ORDER + 1) {
    fir_pwm = new FIR_Transposed<MFIR_ORDER>(mhn);
}

FilteredMotor::~FilteredMotor() {
    delete fir_pwm;
}

void FilteredMotor::setPWM(int pwm) {
    /* process pwm by the Low Pass Filter */
    filtered_pwm = fir_pwm->Execute(pwm);
    /* decrement counter */
    //fillFIR--;
    /* pass through until FIR array is filled */
    if (fillFIR > 0) {
        ev3api::Motor::setPWM(pwm);
    } else {
        ev3api::Motor::setPWM(filtered_pwm);
    }
}