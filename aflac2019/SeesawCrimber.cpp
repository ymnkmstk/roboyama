//
//  SeesawCrimber.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "crew.hpp"

SeesawCrimber::SeesawCrimber(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs) {
    _debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    tailMotor   = tm;
    gyroSensor  = gs;
    colorSensor = cs;
}

void SeesawCrimber::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08lu, SeesawCrimber has control", clock->now());
}

void SeesawCrimber::operate() {
}

SeesawCrimber::~SeesawCrimber() {
    _debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber destructor", clock->now()));
}
