//
//  LimboDancer.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "crew.hpp"

LimboDancer::LimboDancer(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs) {
    _debug(syslog(LOG_NOTICE, "%08lu, LimboDancer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    tailMotor   = tm;
    gyroSensor  = gs;
    colorSensor = cs;
}

void LimboDancer::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08lu, LimboDancer has control", clock->now());
}

void LimboDancer::operate() {
}

LimboDancer::~LimboDancer() {
    _debug(syslog(LOG_NOTICE, "%08lu, LimboDancer destructor", clock->now()));
}
