//
//  SeesawCrimber.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "crew.hpp"

SeesawCrimber::SeesawCrimber(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs) : LineTracer(lm, rm, tm, gs, cs) {
    _debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber constructor", clock->now()));
}

void SeesawCrimber::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08lu, SeesawCrimber has control", clock->now());
}

void SeesawCrimber::operate() {
    LineTracer::operate();
}

SeesawCrimber::~SeesawCrimber() {
    _debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber destructor", clock->now()));
}
