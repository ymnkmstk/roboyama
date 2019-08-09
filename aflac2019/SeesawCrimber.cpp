//
//  SeesawCrimber.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "crew.hpp"
#include <string.h>
#include <stdlib.h>


SeesawCrimber::SeesawCrimber(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs) : LineTracer(lm, rm, tm, gs, cs) {
	readPropFile("/ev3rt/res/Seesaw_prop.txt");
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
		s_angle = (int16_t)getProp( "SEESAW_ANGLE_01" );
		break;
	case SEESAW_01:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_02;
			s_counter = 0;
		}
		pwm_L = 15;
		pwm_R = 15;
		s_angle = 80;
		break;
	case SEESAW_02:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_03;
			s_counter = 0;
		}
		pwm_L = 15;
		pwm_R = 15;
		s_angle = 60;
		break;

	case SEESAW_03:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_04;
			s_counter = 0;
		}
		pwm_L = 0;
		pwm_R = 0;
		s_angle = 60;
		break;

	case SEESAW_04:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_05;
			s_counter = 0;
		}
		pwm_L = -15;
		pwm_R = -15;
		s_angle = 60;
		break;

	case SEESAW_05:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_06;
			s_counter = 0;
		}
		pwm_L = -15;
		pwm_R = -15;
		s_angle = 80;
		break;

	case SEESAW_06:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_07;
			s_counter = 0;
		}
		pwm_L = 0;
		pwm_R = 0;
		s_angle = 80;
		break;

	case SEESAW_07:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_08;
			s_counter = 0;
		}
		pwm_L = 15;
		pwm_R = 15;
		s_angle = 80;
		break;

	case SEESAW_08:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_09;
			s_counter = 0;
		}
		pwm_L = 15;
		pwm_R = 15;
		s_angle = 80;
		break;

	case SEESAW_09:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_10;
			s_counter = 0;
		}
		pwm_L = 0;
		pwm_R = 0;
		s_angle = 80;
		break;

	case SEESAW_10:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_11;
			s_counter = 0;
		}
		pwm_L = 15;
		pwm_R = 15;
		s_angle = 80;
		break;

	case SEESAW_11:
		if ( ++s_counter > PERIOD_SEESAW ){
			s_mode = SEESAW_12;
			s_counter = 0;
		}
		pwm_L = 0;
		pwm_R = 0;
		s_angle = 80;
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
		_debug(syslog(LOG_NOTICE, "%08u, SeesawCrimber::operate(): prop = %d", clock->now(), (int16_t)getProp( "SEESAW_ANGLE_01" ) ));

	}
}

int SeesawCrimber::readLine( FILE* file, char* dst, size_t len ){
    int c = 0;
    unsigned int i = 0;

    while ((c = fgetc(file)) != EOF) {
      if (c < 0) {
        return c;        // error handling
      }
      if ( i + 1 == len ) {  // reached end of buffer
        dst[i] = '\0';
        break;
      }
      if (c == '\n') {   // reached end of line
        dst[i] = '\0';
        break;
      }
      dst[i] = (char)c;
      i++;
    }
    return i;
}

void SeesawCrimber::readPropFile( const char* filename ){

	FILE* prop_file = NULL;
	prop_file = fopen( filename, "r" );
	if( prop_file==NULL){
		_debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber readProfile() file not found", clock->now()));
		return;
	}


	memset( &props, 0, sizeof(struct property) * NUM_PROPS );

	int retval = 0;
	int i = 0;
	char buf[256];
	memset( &buf, 0, sizeof(buf) );

	while( ( retval = readLine(prop_file, buf, 256)) > 0 ){
		char* comptr = strstr( buf, "," );
		if( i < NUM_PROPS && NULL != comptr ){
			*comptr = '\0';
			strcpy( props[i].name, buf );
			props[i].value = atoi(comptr+1);
			i++;
		}else
			break;
	}
	fclose( prop_file );

}

int SeesawCrimber::getProp( const char* propname ){
	if( props == NULL ){
		return 0;
	}

	for ( int i = 0; i < NUM_PROPS; i++ ){
		if( strcmp( propname, props[i].name ) == 0 ){
			return props[i].value;
		}
	}
	return 0;
}

SeesawCrimber::~SeesawCrimber() {
    _debug(syslog(LOG_NOTICE, "%08lu, SeesawCrimber destructor", clock->now()));
}
