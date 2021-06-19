/*
    FilteredMotor.cpp

    Copyright © 2021 MS Mode 2. All rights reserved.
*/
#include "FilteredMotor.hpp"

FilteredMotor::FilteredMotor(ePortM port) : Motor(port),fil(nullptr) {}

void FilteredMotor::setPWMFilter(Filter *filter) {
    fil = filter;
}

void FilteredMotor::drive() {
    int filtered_pwm;
    /* process pwm by the Filter */
    if (fil == nullptr) {
        filtered_pwm = original_pwm;
    } else {
        filtered_pwm = fil->apply(original_pwm);
    }
    ev3api::Motor::setPWM(filtered_pwm);
}