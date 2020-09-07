//
//  Captain.cpp
//  aflac2020
//
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "crew.hpp"

Captain::Captain() {
    _debug(syslog(LOG_NOTICE, "%08u, Captain default constructor", clock->now()));
}

void Captain::takeoff() {
    /* 各オブジェクトを生成・初期化する */
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_2);
    colorSensor = new ColorSensor(PORT_3);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_A);
    steering    = new Steering(*leftMotor, *rightMotor);
    
    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET aflac2020", 0, CALIB_FONT_HEIGHT*1);
    
    observer = new Observer(leftMotor, rightMotor, touchSensor, sonarSensor, gyroSensor, colorSensor);
    observer->freeze(); // Do NOT attempt to collect sensor data until unfreeze() is invoked
    observer->goOnDuty();
    blindRunner = new BlindRunner(leftMotor, rightMotor, tailMotor);
    lineTracer = new LineTracer(leftMotor, rightMotor, tailMotor);
    lineTracer->goOnDuty();
    
    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    state = ST_takingOff;
}

void Captain::decide(uint8_t event) {
    syslog(LOG_NOTICE, "%08u, Captain::decide(): event %s received by state %s", clock->now(), eventName[event], stateName[state]);
    switch (state) {
        case ST_takingOff:
            switch (event) {
                case EVT_cmdStart_R:
                case EVT_cmdStart_L:
                case EVT_touch_On:
                    if (event == EVT_cmdStart_L || (event == EVT_touch_On && _LEFT)) {
                        state = ST_tracing_L;
                    } else {  // event == EVT_cmdStart_R || (event == EVT_touch_On && !_LEFT)
                        state = ST_tracing_R;
                    }
                    syslog(LOG_NOTICE, "%08u, Departing...", clock->now());
                    
                    /* 走行モーターエンコーダーリセット */
                    leftMotor->reset();
                    rightMotor->reset();
                    
                    //balance_init(); /* 倒立振子API初期化 */
                    observer->reset();
                    
                    /* ジャイロセンサーリセット */
                    gyroSensor->reset();
                    ev3_led_set_color(LED_GREEN); /* スタート通知 */
                    
                    observer->freeze();
                    lineTracer->freeze();
                    lineTracer->haveControl();
                    //clock->sleep() seems to be still taking milisec parm
                    clock->sleep(PERIOD_NAV_TSK*FIR_ORDER/1000); // wait until FIR array is filled
                    lineTracer->unfreeze();
                    observer->unfreeze();
                    syslog(LOG_NOTICE, "%08u, Departed", clock->now());
                   break;
                default:
                    break;
            }
            break;
        case ST_tracing_R:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    triggerLanding();
                    break;
                case EVT_sonar_On:
		    //lineTracer->freeze();
		    // During line trancing,
		    // if sonar is on (limbo sign is near by matchine),
		    // limbo dance starts.
                    //state = ST_dancing;
                    //limboDancer->haveControl();
                    break;
                case EVT_sonar_Off:
                    //lineTracer->unfreeze();
                    break;
                case EVT_cmdDance:
                case EVT_bl2bk:
                    //state = ST_dancing;
                    //limboDancer->haveControl();
                    break;
                case EVT_bk2bl:
                    /*
                    // stop at the start of blue line
                    observer->freeze();
                    lineTracer->freeze();
                    //clock->sleep() seems to be still taking milisec parm
                    clock->sleep(5000); // wait a little
                    lineTracer->unfreeze();
                    observer->unfreeze();
                    */
                    break;
                case EVT_cmdStop:
                    state = ST_stopping_R;
                    observer->notifyOfDistance(FINAL_APPROACH_LEN);
                    lineTracer->haveControl();
                    break;
                default:
                    break;
            }
            break;
        case ST_tracing_L:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    triggerLanding();
                    break;
                case EVT_sonar_On:
                    //lineTracer->freeze();
                    break;
                case EVT_sonar_Off:
                    //lineTracer->unfreeze();
                    break;
                case EVT_cmdCrimb:
                case EVT_bl2bk:
                    //state = ST_crimbing;
                    //seesawCrimber->haveControl();
                    break;
                case EVT_bk2bl:
                    /*
                    // stop at the start of blue line
                    observer->freeze();
                    lineTracer->freeze();
                    //clock->sleep() seems to be still taking milisec parm
                    clock->sleep(5000); // wait a little
                    lineTracer->unfreeze();
                    observer->unfreeze();
                    */
                    break;
                case EVT_cmdStop:
                    state = ST_stopping_L;
                    observer->notifyOfDistance(FINAL_APPROACH_LEN);
                    lineTracer->haveControl();
                    break;
                default:
                    break;
            }
            break;
        case ST_dancing:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    triggerLanding();
                    break;
                case EVT_bk2bl:
		    // Don't use "black line to blue line" event.
  		    /*
                    state = ST_stopping_R;
                    observer->notifyOfDistance(FINAL_APPROACH_LEN);
                    lineTracer->haveControl();
		    */
                    break;
                default:
                    break;
            }
            break;
        case ST_crimbing:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    triggerLanding();
                    break;
                case EVT_bk2bl:
                    state = ST_stopping_L;
                    observer->notifyOfDistance(FINAL_APPROACH_LEN);
                    lineTracer->haveControl();
                    break;
                default:
                    break;
            }
            break;
        case ST_stopping_R:
        case ST_stopping_L:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    triggerLanding();
                    break;
                case EVT_dist_reached:
                    state = ST_landing;
                    triggerLanding();
                    break;
                default:
                    break;
            }
            break;
        case ST_landing:
            break;
        default:
            break;
    }
}

void Captain::triggerLanding() {
    syslog(LOG_NOTICE, "%08u, Landing...", clock->now());
    ER ercd = wup_tsk(MAIN_TASK); // wake up the main task
    assert(ercd == E_OK);
}

void Captain::land() {
    if (activeNavigator != NULL) {
        activeNavigator->goOffDuty();
    }
    leftMotor->reset();
    rightMotor->reset();
    
    delete lineTracer;
    delete blindRunner;
    observer->goOffDuty();
    delete observer;
    
    delete tailMotor;
    delete rightMotor;
    delete leftMotor;
    delete gyroSensor;
    delete colorSensor;
    delete sonarSensor;
    delete touchSensor;
}

Captain::~Captain() {
    _debug(syslog(LOG_NOTICE, "%08u, Captain destructor", clock->now()));
}
