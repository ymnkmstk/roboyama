/*
    FilteredMotor.cpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#include "FilteredMotor.hpp"
constexpr const double FilteredMotor::hn[FIR_ORDER+1];

FilteredMotor::FilteredMotor(ePortM port) : Motor(port), fillFIR(FIR_ORDER + 1) {
    fir_pwm = new FIR_Transposed<FIR_ORDER>(hn);
}

FilteredMotor::~FilteredMotor() {
    delete fir_pwm;
}

void FilteredMotor::setPWM(int pwm) {
    /* process pwm by the Low Pass Filter */
    filtered_pwm = fir_pwm->Execute(pwm);
    /* decrement counter */
    fillFIR--;
    /* pass through until FIR array is filled */
    if (fillFIR > 0) {
        ev3api::Motor::setPWM(pwm);
    } else {
        ev3api::Motor::setPWM(filtered_pwm);
    }
}

int32_t FilteredMotor::getPwm(){
    return ev3api::Motor::getPWM();
}

