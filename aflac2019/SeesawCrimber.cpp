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
}

void SeesawCrimber::haveControl() {
    activeNavigator = this;

    // private変数の初期化
    s_time = 0;
    s_counter = 0;
    s_speed = 0;
    s_mode = SEESAW_00;
    s_angle = TAIL_ANGLE_NORMAL_RUN;

    // ログ出力
    syslog(LOG_NOTICE, "%08lu, SeesawCrimber has control", clock->now());
}

void SeesawCrimber::operate() {

	// 初期状態での尻尾角度をTAIL_ANGLE_NORMAL_RUNに合わせて設定。
	controlTail(TAIL_ANGLE_NORMAL_RUN);

	switch( s_mode ){
	case SEESAW_00:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_01;
			s_counter = 0;
		}
		pwm_L = 0;
		pwm_R = 0;
		s_angle = 80;
		break;
	case SEESAW_01:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_02;
			s_counter = 0;
		}
		pwm_L = 50;
		pwm_R = 50;
		s_angle = 80;
		break;
	case SEESAW_02:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_03;
			s_counter = 0;
		}
		pwm_L = 15;
		pwm_R = 15;
		s_angle = 50;
		break;

	case SEESAW_03:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_04;
			s_counter = 0;
		}
		pwm_L = 0;
		pwm_R = 0;
		s_angle = 40;
		break;

	case SEESAW_04:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_05;
			s_counter = 0;
		}
		pwm_L = -15;
		pwm_R = -15;
		s_angle = 40;
		break;

	case SEESAW_05:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_06;
			s_counter = 0;
		}
		pwm_L = -15;
		pwm_R = -15;
		s_angle = 50;
		break;

	case SEESAW_06:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_07;
			s_counter = 0;
		}
		pwm_L = 0;
		pwm_R = 0;
		s_angle = 50;
		break;

	case SEESAW_07:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_08;
			s_counter = 0;
		}
		pwm_L = 15;
		pwm_R = 15;
		s_angle = 50;
		break;

	case SEESAW_08:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_09;
			s_counter = 0;
		}
		pwm_L = 15;
		pwm_R = 15;
		s_angle = 40;
		break;

	case SEESAW_09:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_10;
			s_counter = 0;
		}
		pwm_L = 0;
		pwm_R = 0;
		s_angle = 40;
		break;

	case SEESAW_10:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_11;
			s_counter = 0;
		}
		pwm_L = 15;
		pwm_R = 15;
		s_angle = 40;
		break;

	case SEESAW_11:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_12;
			s_counter = 0;
		}
		pwm_L = 0;
		pwm_R = 0;
		s_angle = 60;
		break;

	case SEESAW_12:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_01;
			s_counter = 0;
		}
		pwm_L = 0;
		pwm_R = 0;
		s_angle = 80;
		break;

	case SEESAW_13:
		pwm_L = 15;
		pwm_R = 15;
		s_angle = 80;
		break;

	}

	// 左右モーターと尻尾モーターへ値を渡す
	leftMotor->setPWM(pwm_L);
	rightMotor->setPWM(pwm_R);
	controlTail(s_angle);

	// ログを PERIOD_TRAVE_MSG ms で出力する
	if (++s_trace_counter * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG ) {
		s_trace_counter = 0;
		_debug(syslog(LOG_NOTICE, "%08u, SeesawCrimber::operate(): case_no = %d, tail_angle = %d", clock->now(), s_mode, s_angle));
		_debug(syslog(LOG_NOTICE, "%08u, SeesawCrimber::operate(): pwm_L = %d, pwm_R = %d", clock->now(), pwm_L, pwm_R));
	}
}

SeesawCrimber::~SeesawCrimber() {
    _debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber destructor", clock->now()));
}
