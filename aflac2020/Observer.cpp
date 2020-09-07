//
//  Observer.cpp
//  aflac2020
//
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "Observer.hpp"
#include "StateMachine.hpp"

// global variables to pass FIR-filtered color from Observer to Navigator and its sub-classes
rgb_raw_t g_rgb;
hsv_raw_t g_hsv;
int16_t g_grayScale, g_grayScaleBlueless;
// global variables to gyro sensor output from Observer to  Navigator and its sub-classes
int16_t g_angle, g_anglerVelocity;

Observer::Observer(Motor* lm, Motor* rm, TouchSensor* ts, SonarSensor* ss, GyroSensor* gs, ColorSensor* cs) {
    _debug(syslog(LOG_NOTICE, "%08u, Observer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    touchSensor = ts;
    sonarSensor = ss;
    gyroSensor  = gs;
    colorSensor = cs;
    distance = 0.0;
    azimuth = 0.0;
    locX = 0.0;
    locY = 0.0;
    prevAngL = 0;
    prevAngR = 0;
    notifyDistance = 0;
    traceCnt = 0;
    prevGS = INT16_MAX;
    touch_flag = false;
    sonar_flag = false;
    backButton_flag = false;
    lost_flag = false;
    blue_flag = false;
    //ot_r = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);
    //ot_g = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);
    //ot_b = new OutlierTester(OLT_SKIP_PERIOD/PERIOD_OBS_TSK, OLT_INIT_PERIOD/PERIOD_OBS_TSK);

    fir_r = new FIR_Transposed<FIR_ORDER>(hn);
    fir_g = new FIR_Transposed<FIR_ORDER>(hn);
    fir_b = new FIR_Transposed<FIR_ORDER>(hn);
    ma = new MovingAverage<int32_t, MA_CAP>();
}

void Observer::activate() {
    // register cyclic handler to EV3RT
    sta_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Observer handler set", clock->now()));
}

void Observer::reset() {
    distance = 0.0;
    azimuth = 0.0;
    locX = 0.0;
    locY = 0.0;
    prevAngL = leftMotor->getCount();
    prevAngR = rightMotor->getCount();
}

void Observer::notifyOfDistance(int32_t delta) {
    notifyDistance = delta + distance;
}

int32_t Observer::getDistance() {
    return (int32_t)distance;
}

int16_t Observer::getAzimuth() {
    // degree = 360.0 * radian / M_2PI;
    int16_t degree = (360.0 * azimuth / M_2PI);
    return degree;
}

int32_t Observer::getLocX() {
    return (int32_t)locX;
}

int32_t Observer::getLocY() {
    return (int32_t)locY;
}

void Observer::operate() {
    colorSensor->getRawColor(cur_rgb);
    // process RGB by the Low Pass Filter
    cur_rgb.r = fir_r->Execute(cur_rgb.r);
    cur_rgb.g = fir_g->Execute(cur_rgb.g);
    cur_rgb.b = fir_b->Execute(cur_rgb.b);
    rgb_to_hsv(cur_rgb, cur_hsv);
    // save filtered color variables to the global area
    g_rgb = cur_rgb;
    g_hsv = cur_hsv;
    // calculate gray scale and save them to the global area
    g_grayScale = (cur_rgb.r * 77 + cur_rgb.g * 150 + cur_rgb.b * 29) / 256;
    g_grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 150 + (cur_rgb.b - cur_rgb.g) * 29) / 256; // B - G cuts off blue
    // save gyro sensor output to the global area
    g_angle = gyroSensor->getAngle();
    g_anglerVelocity = gyroSensor->getAnglerVelocity();

    // accumulate distance
    int32_t curAngL = leftMotor->getCount();
    int32_t curAngR = rightMotor->getCount();
    double deltaDistL = M_PI * TIRE_DIAMETER * (curAngL - prevAngL) / 360.0;
    double deltaDistR = M_PI * TIRE_DIAMETER * (curAngR - prevAngR) / 360.0;
    double deltaDist = (deltaDistL + deltaDistR) / 2.0;
    distance += deltaDist;
    prevAngL = curAngL;
    prevAngR = curAngR;
    // calculate azimuth
    double deltaAzi = atan2((deltaDistL - deltaDistR), WHEEL_TREAD);
    azimuth += deltaAzi;
    if (azimuth > M_2PI) {
        azimuth -= M_2PI;
    } else if (azimuth < 0.0) {
        azimuth += M_2PI;
    }
    // estimate location
    locX += (deltaDist * sin(azimuth));
    locY += (deltaDist * cos(azimuth));

    // monitor distance
    if ((notifyDistance != 0.0) && (distance > notifyDistance)) {
        syslog(LOG_NOTICE, "%08u, distance reached", clock->now());
        notifyDistance = 0.0; // event to be sent only once
        stateMachine->sendTrigger(EVT_dist_reached);
    }
    
    // monitor touch sensor
    bool result = check_touch();
    if (result && !touch_flag) {
        syslog(LOG_NOTICE, "%08u, TouchSensor flipped on", clock->now());
        touch_flag = true;
        stateMachine->sendTrigger(EVT_touch_On);
    } else if (!result && touch_flag) {
        syslog(LOG_NOTICE, "%08u, TouchSensor flipped off", clock->now());
        touch_flag = false;
        stateMachine->sendTrigger(EVT_touch_Off);
    }
    
    // monitor sonar sensor
    result = check_sonar();
    if (result && !sonar_flag) {
        syslog(LOG_NOTICE, "%08u, SonarSensor flipped on", clock->now());
        sonar_flag = true;
        stateMachine->sendTrigger(EVT_sonar_On);
    } else if (!result && sonar_flag) {
        syslog(LOG_NOTICE, "%08u, SonarSensor flipped off", clock->now());
        sonar_flag = false;
        stateMachine->sendTrigger(EVT_sonar_Off);
    }
    
    // monitor Back Button
    result = check_backButton();
    if (result && !backButton_flag) {
        syslog(LOG_NOTICE, "%08u, Back button flipped on", clock->now());
        backButton_flag = true;
        stateMachine->sendTrigger(EVT_backButton_On);
    } else if (!result && backButton_flag) {
        syslog(LOG_NOTICE, "%08u, Back button flipped off", clock->now());
        backButton_flag = false;
        stateMachine->sendTrigger(EVT_backButton_Off);
    }

    if (!frozen) { // these checks are meaningless thus bypassed when frozen
        // determine if still tracing the line
        result = check_lost();
        if (result && !lost_flag) {
            syslog(LOG_NOTICE, "%08u, line lost", clock->now());
            lost_flag = true;
            stateMachine->sendTrigger(EVT_line_lost);
        } else if (!result && lost_flag) {
            syslog(LOG_NOTICE, "%08u, line found", clock->now());
            lost_flag = false;
            stateMachine->sendTrigger(EVT_line_found);
        }

        // temporary dirty logic to detect the second black to blue change
        int32_t ma_gs;
        if (prevGS == INT16_MAX) {
            prevTime = clock->now();
            prevGS = g_grayScale;
            ma_gs = ma->add(0);
        } else {
            curTime = clock->now();
            gsDiff = g_grayScale - prevGS;
            timeDiff = curTime - prevTime;
            ma_gs = ma->add(gsDiff * 1000000 / timeDiff);
            prevTime = curTime;
            prevGS = g_grayScale;
        }

        if ( (ma_gs > 150) || (ma_gs < -150) ){
            //syslog(LOG_NOTICE, "gs = %d, MA = %d, gsDiff = %d, timeDiff = %d", g_grayScale, ma_gs, gsDiff, timeDiff);
            if ( !blue_flag && ma_gs > 150 && g_rgb.b - g_rgb.r > 60 && g_rgb.b <= 255 && g_rgb.r <= 255 ) {
                blue_flag = true;
                syslog(LOG_NOTICE, "%08u, line color changed black to blue", clock->now());
                stateMachine->sendTrigger(EVT_bk2bl);
            } else if ( blue_flag && ma_gs < -150 && g_rgb.b - g_rgb.r < 40 ) {
                blue_flag = false;
                syslog(LOG_NOTICE, "%08u, line color changed blue to black", clock->now());
                stateMachine->sendTrigger(EVT_bl2bk);
            }
        }

        // determine if tilt
        if ( check_tilt() ) {
            //stateMachine->sendTrigger(EVT_cmdStop);
        }
    }
    
    /*
    int32_t iDeltaD  = (int32_t)(1000.0 * deltaDist );
    int32_t iDeltaDL = (int32_t)(1000.0 * deltaDistL);
    int32_t iDeltaDR = (int32_t)(1000.0 * deltaDistR);
    if (iDeltaDL != 0 && iDeltaDR != 0) {
        _debug(syslog(LOG_NOTICE, "%08u, %06d, %06d, %06d, %06d", clock->now(), getDistance(), iDeltaD, iDeltaDL, iDeltaDR));
    }
    */

    /*
    // display trace message in every PERIOD_TRACE_MSG ms
    if (++traceCnt * PERIOD_OBS_TSK >= PERIOD_TRACE_MSG) {
        traceCnt = 0;
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), getDistance(), getAzimuth(), getLocX(), getLocY()));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): hsv = (%03u, %03u, %03u)", clock->now(), g_hsv.h, g_hsv.s, g_hsv.v));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): rgb = (%03u, %03u, %03u)", clock->now(), g_rgb.r, g_rgb.g, g_rgb.b));
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): angle = %d, anglerVelocity = %d", clock->now(), g_angle, g_anglerVelocity));
    }
    */
}

void Observer::deactivate() {
    // deregister cyclic handler from EV3RT
    stp_cyc(CYC_OBS_TSK);
    //clock->sleep() seems to be still taking milisec parm
    clock->sleep(PERIOD_OBS_TSK/2/1000); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Observer handler unset", clock->now()));
}

bool Observer::check_touch(void) {
    if (touchSensor->isPressed()) {
        return true;
    } else {
        return false;
    }
}

bool Observer::check_sonar(void) {
    int32_t distance = sonarSensor->getDistance();
    if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0)) {
        return true; // obstacle detected - alert
    } else {
        return false; // no problem
    }
}

bool Observer::check_backButton(void) {
    if (ev3_button_is_pressed(BACK_BUTTON)) {
        return true;
    } else {
        return false;
    }
}

bool Observer::check_lost(void) {
    if (g_grayScale > GS_LOST) {
        return true;
    } else {
        return false;
    }
    /*
    int8_t otRes_r, otRes_g, otRes_b;
    otRes_r = ot_r->test(cur_rgb.r);
    otRes_g = ot_g->test(cur_rgb.g);
    otRes_b = ot_b->test(cur_rgb.b);
    if ((otRes_r == POS_OUTLIER && otRes_g == POS_OUTLIER) ||
        (otRes_g == POS_OUTLIER && otRes_b == POS_OUTLIER) ||
        (otRes_b == POS_OUTLIER && otRes_r == POS_OUTLIER)) {
        return true;
    } else {
        return false;
    }
    */
}

bool Observer::check_tilt(void) {
    int16_t anglerVelocity = gyroSensor->getAnglerVelocity();
    if (anglerVelocity < ANG_V_TILT && anglerVelocity > (-1) * ANG_V_TILT) {
        return false;
    } else {
        _debug(syslog(LOG_NOTICE, "%08u, Observer::operate(): TILT anglerVelocity = %d", clock->now(), anglerVelocity));
        return true;
    }
}

void Observer::freeze() {
    frozen = true;
}

void Observer::unfreeze() {
    frozen = false;
}

Observer::~Observer() {
    _debug(syslog(LOG_NOTICE, "%08u, Observer destructor", clock->now()));
}