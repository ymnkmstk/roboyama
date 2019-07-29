//
//  LimboDancer.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "crew.hpp"

LimboDancer::LimboDancer(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs) : LineTracer(lm, rm, tm, gs, cs) {
    limboMode = 1;
    counter = 0;
    _debug(syslog(LOG_NOTICE, "%08lu, LimboDancer constructor", clock->now()));
}

void LimboDancer::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08lu, LimboDancer has control", clock->now());
}

void LimboDancer::operate() {
    _debug(syslog(LOG_NOTICE, "%08u, LimboDancer::operate(): distance = %u, mode = %d", clock->now(), observer->getDistance(), limboMode));
    if ( limboMode == 1 ) { // Dash small distance and start Limbo
        if ( ++counter > 30 ) {
	  limboMode = 2;
	}
        forward = 30;
	turn = 0;
    } else if ( limboMode == 2 ) { // Forward with slow speed by Limbo Style
        controlTail(TAIL_ANGLE_LIMBO);
	if ( ++counter > 1030 ) {
	  limboMode = 3;
	}
        forward = 10;
	turn = 0;
    } else if ( limboMode == 3 ) { // Back with slow speed by Limbo Style
        controlTail(TAIL_ANGLE_LIMBO);
	if ( ++counter > 2030 ) {
	  limboMode = 4;
	}
        forward = -10;
	turn = 0;
    } else if ( limboMode == 4 ) { // forward with slow speed by Limbo Style
        controlTail(TAIL_ANGLE_LIMBO);
	if ( ++counter > 3030 ) {
	  limboMode = 5;
	}
        forward = 10;
	turn = 0;
    } else /* limboMode == 5 */ { // stop by Limbo Style
        controlTail(TAIL_ANGLE_LIMBO);
        forward = 0;
	turn = 0;
    }
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

LimboDancer::~LimboDancer() {
    _debug(syslog(LOG_NOTICE, "%08lu, LimboDancer destructor", clock->now()));
}
