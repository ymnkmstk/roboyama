//
//  SeesawCrimber.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "crew.hpp"

SeesawCrimber::SeesawCrimber(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs) : LineTracer(lm, rm, tm, gs, cs) {
    _debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber constructor", clock->now()));
    startTime=0;
    counter=0;
    speed=0;
}

void SeesawCrimber::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08lu, SeesawCrimber has control", clock->now());
}

void SeesawCrimber::operate() {
	_debug(syslog(LOG_NOTICE,"%08lu, SeesawCrimber operate start", clock->now()));
	controlTail(TAIL_ANGLE_NORMAL_RUN);

	forward = START_SPEED;
	turn = 0;

	pwm_L = forward;
	pwm_R = forward;

	leftMotor->setPWM(pwm_L);
	rightMotor->setPWM(pwm_R);

	// display pwm in every PERIOD_TRACE_MSG ms */
	    if (++trace_pwmLR * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG) {
	        trace_pwmLR = 0;
	        _debug(syslog(LOG_NOTICE, "%08u, LimboDancer::operate(): pwm_L = %d, pwm_R = %d", clock->now(), pwm_L, pwm_R));
	    }
}

SeesawCrimber::~SeesawCrimber() {
    _debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber destructor", clock->now()));
}
