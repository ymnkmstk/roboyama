//
//  crew.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/04/28.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"
#include "balancer.h"
#include "crew.hpp"

/* Observer event flags */
/* ToDo: replace these by priorityQueue */
bool bt_flag         = false; /* Bluetoothコマンド true:リモートスタート */
bool touch_flag      = false; /* TouchSensor true:タッチセンサー押下 */
bool sonar_flag      = false; /* SonarSensor true:障害物検知 */
bool backButton_flag = false; /* true: BackButton押下 */

Observer::Observer(Motor* lm, Motor* rm, TouchSensor* ts,SonarSensor* ss) {
    _debug(syslog(LOG_NOTICE, "%08u, Observer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    touchSensor = ts;
    sonarSensor = ss;
    bt = NULL;
    distance = 0.0;
    azimuth = 0.0;
    locX = 0.0;
    locY = 0.0;
    prevAngL = 0;
    prevAngR = 0;
}

void Observer::goOnDuty() {
    /* Open Bluetooth file */
    /*
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);
    */

    // register cyclic handler to EV3RT
    ev3_sta_cyc(CYC_OBS_TSK);
    clock->sleep(PERIOD_OBS_TSK/2); // wait a while
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

    /*
    if (check_bt() && !bt_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, StartCMD received", clock->now()));
        bt_flag = true;
    }
    */
    
    if (check_touch() && !touch_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, TouchSensor flipped on", clock->now()));
        touch_flag = true;
    } else if (!check_touch() && touch_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, TouchSensor flipped off", clock->now()));
        touch_flag = false;
    }
    if (check_sonar() && !sonar_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, SonarSensor flipped on", clock->now()));
        sonar_flag = true;
    } else if (!check_sonar() && sonar_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, SonarSensor flipped off", clock->now()));
        sonar_flag = false;
    }
    if (check_backButton() && !backButton_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, Back button flipped on", clock->now()));
        backButton_flag = true;
    } else if (!check_backButton() && backButton_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, Back button flipped off", clock->now()));
        backButton_flag = false;
    }
}

void Observer::goOffDuty() {
    // deregister cyclic handler from EV3RT
    ev3_stp_cyc(CYC_OBS_TSK);
    clock->sleep(PERIOD_OBS_TSK/2); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Observer handler unset", clock->now()));
    
    //fclose(bt);
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

bool Observer::check_bt(void) {
    uint8_t c = fgetc(bt); /* 受信 */
    fputc(c, bt); /* エコーバック */
    switch(c)
    {
        case '1':
            return true;
        default:
            return false;
    }
}

bool Observer::check_backButton(void) {
    if (ev3_button_is_pressed(BACK_BUTTON)) {
        return true;
    } else {
        return false;
    }
}

Observer::~Observer() {
    _debug(syslog(LOG_NOTICE, "%08u, Observer destructor", clock->now()));
}

Navigator::Navigator() {
    _debug(syslog(LOG_NOTICE, "%08u, Navigator default constructor", clock->now()));
    setPIDconst(0.38, 0.06, 0.027); // set default PID constant
    diff[1] = 0; // initialize diff[1]
}

//*****************************************************************************
// 引数 : lpwm (左モーターPWM値 ※前回の出力値)
//        rpwm (右モーターPWM値 ※前回の出力値)
//        lenc (左モーターエンコーダー値)
//        renc (右モーターエンコーダー値)
// 返り値 : なし
// 概要 : 直近のPWM値に応じてエンコーダー値にバックラッシュ分の値を追加します。
//*****************************************************************************
void Navigator::cancelBacklash(int8_t lpwm, int8_t rpwm, int32_t *lenc, int32_t *renc) {
    const int32_t BACKLASHHALF = 4;   // バックラッシュの半分[deg]
    
    if(lpwm < 0) *lenc += BACKLASHHALF;
    else if(lpwm > 0) *lenc -= BACKLASHHALF;
    
    if(rpwm < 0) *renc += BACKLASHHALF;
    else if(rpwm > 0) *renc -= BACKLASHHALF;
}

//*****************************************************************************
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void Navigator::controlTail(int32_t angle) {
    float pwm = (float)(angle - tailMotor->getCount()) * P_GAIN; /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX) {
        pwm = PWM_ABS_MAX;
    } else if (pwm < -PWM_ABS_MAX) {
        pwm = -PWM_ABS_MAX;
    }

    tailMotor->setPWM(pwm);

    // display pwm in every PERIOD_TRACE_MSG ms */
    if (++trace_pwmT * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG) {
       trace_pwmT = 0;
        _debug(syslog(LOG_NOTICE, "%08u, Navigator::controlTail(): pwm = %d", clock->now(), pwm));
    }
}

void Navigator::setPIDconst(long double p, long double i, long double d) {
    kp = p;
    ki = i;
    kd = d;
}

int8_t Navigator::math_limit(int8_t input, int8_t min, int8_t max) {
    if (input < min) {
        return min;
    } else if (input > max) {
        return max;
    }
    return input;
}

int8_t Navigator::computePID(int8_t sensor, int8_t target) {
    long double p, i, d;
    
    diff[0] = diff[1];
    diff[1] = sensor - target;
    integral += (diff[0] + diff[1]) / 2.0 * PERIOD_NAV_TSK / 1000;
    
    p = kp * diff[1];
    i = ki * integral;
    d = kd * (diff[1] - diff[0]) * 1000 / PERIOD_NAV_TSK;
    return math_limit(p + i + d, -100.0, 100.0);
}

void Navigator::goOnDuty() {
    // register cyclic handler to EV3RT
    ev3_sta_cyc(CYC_NAV_TSK);
    clock->sleep(PERIOD_NAV_TSK/2); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Navigator handler set", clock->now()));
}

void Navigator::goOffDuty() {
    activeNavigator = NULL;
    // deregister cyclic handler from EV3RT
    ev3_stp_cyc(CYC_NAV_TSK);
    clock->sleep(PERIOD_NAV_TSK/2); // wait a while
    _debug(syslog(LOG_NOTICE, "%08u, Navigator handler unset", clock->now()));
}

Navigator::~Navigator() {
    _debug(syslog(LOG_NOTICE, "%08u, Navigator destructor", clock->now()));
}

AnchorWatch::AnchorWatch(Motor* tm) {
    _debug(syslog(LOG_NOTICE, "%08u, AnchorWatch constructor", clock->now()));
    tailMotor   = tm;
    trace_pwmT  = 0;
}

void AnchorWatch::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08u, AnchorWatch has control", clock->now());
}

void AnchorWatch::operate() {
    controlTail(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */
}

AnchorWatch::~AnchorWatch() {
    _debug(syslog(LOG_NOTICE, "%08u, AnchorWatch destructor", clock->now()));
}

LineTracer::LineTracer(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs) {
    _debug(syslog(LOG_NOTICE, "%08u, LineTracer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    tailMotor   = tm;
    gyroSensor  = gs;
    colorSensor = cs;
    trace_pwmT  = 0;
    trace_pwmLR = 0;
}

void LineTracer::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08u, LineTracer has control", clock->now());
}

void LineTracer::operate() {
    controlTail(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

    if (sonar_flag) {
        forward = turn = 0; /* 障害物を検知したら停止 */
    } else {
        forward = 30; //前進命令
        /*
        // on-off control
        if (colorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK)/2) {
            turn =  20; // 左旋回命令
        } else {
            turn = -20; // 右旋回命令
        }
        */
        int8_t sensor = colorSensor->getBrightness();
        int8_t target = (LIGHT_WHITE + LIGHT_BLACK)/2;
        turn = computePID(sensor, target);
    }
    /* 倒立振子制御API に渡すパラメータを取得する */
    motor_ang_l = leftMotor->getCount();
    motor_ang_r = rightMotor->getCount();
    gyro = gyroSensor->getAnglerVelocity();
    volt = ev3_battery_voltage_mV();

    /* バックラッシュキャンセル */
    cancelBacklash(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);
    
    /* 倒立振子制御APIを呼び出し、倒立走行するための */
    /* 左右モータ出力値を得る */
    balance_control((float)forward,
                    (float)turn,
                    (float)gyro,
                    (float)GYRO_OFFSET,
                    (float)motor_ang_l,
                    (float)motor_ang_r,
                    (float)volt,
                    (int8_t *)&pwm_L,
                    (int8_t *)&pwm_R);

    leftMotor->setPWM(pwm_L);
    rightMotor->setPWM(pwm_R);

    // display pwm in every PERIOD_TRACE_MSG ms */
    if (++trace_pwmLR * PERIOD_NAV_TSK >= PERIOD_TRACE_MSG) {
        trace_pwmLR = 0;
        //_debug(syslog(LOG_NOTICE, "%08u, LineTracer::operate(): pwm_L = %d, pwm_R = %d, distance = %ld, azimuth = %d, x = %ld, y = %ld", clock->now(), pwm_L, pwm_R, observer->getDistance(), observer->getAzimuth(), observer->getLocX(), observer->getLocY()));
        _debug(syslog(LOG_NOTICE, "%08u, LineTracer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), observer->getDistance(), observer->getAzimuth(), observer->getLocX(), observer->getLocY()));
    }
}

LineTracer::~LineTracer() {
    _debug(syslog(LOG_NOTICE, "%08u, LineTracer destructor", clock->now()));
}

HarbourPilot::HarbourPilot(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs) {
    _debug(syslog(LOG_NOTICE, "%08u, HarbourPilot constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    tailMotor   = tm;
    gyroSensor  = gs;
    colorSensor = cs;
}

void HarbourPilot::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08u, HarbourPilot has control", clock->now());
}

void HarbourPilot::operate() {
}

HarbourPilot::~HarbourPilot() {
    _debug(syslog(LOG_NOTICE, "%08u, HarbourPilot destructor", clock->now()));
}

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
    
    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET aflac2019", 0, CALIB_FONT_HEIGHT*1);
    
    // register cyclic handler to EV3RT
    ev3_sta_cyc(CYC_CAP_TSK);
    clock->sleep(PERIOD_CAP_TSK/2); // wait a while

    observer = new Observer(leftMotor, rightMotor, touchSensor, sonarSensor);
    observer->goOnDuty();
    limboDancer = new LimboDancer(leftMotor, rightMotor, tailMotor, gyroSensor, colorSensor);
    seesawCrimber = new SeesawCrimber(leftMotor, rightMotor, tailMotor, gyroSensor, colorSensor);
    lineTracer = new LineTracer(leftMotor, rightMotor, tailMotor, gyroSensor, colorSensor);
    harbourPilot = new HarbourPilot(leftMotor, rightMotor, tailMotor, gyroSensor, colorSensor);
    
    /* 尻尾モーターのリセット */
    tailMotor->reset();
    
    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    anchorWatch = new AnchorWatch(tailMotor);
    anchorWatch->goOnDuty();
    anchorWatch->haveControl();
}

void Captain::operate() {
    /* ToDo: implement a state machine to pick up an appropriate Navigator */
    switch (state) {
        case ST_takingOff:
            if (bt_flag || touch_flag) {
                syslog(LOG_NOTICE, "%08u, Departing...", clock->now());
                
                /* 走行モーターエンコーダーリセット */
                leftMotor->reset();
                rightMotor->reset();
                
                balance_init(); /* 倒立振子API初期化 */
                observer->reset();
                
                /* ジャイロセンサーリセット */
                gyroSensor->reset();
                ev3_led_set_color(LED_GREEN); /* スタート通知 */
                
                lineTracer->haveControl();
                clock->sleep(PERIOD_CAP_TSK/2); // wait a while
                state = ST_tracing;
            }
            break;
        case ST_tracing:
            if (backButton_flag) {
                syslog(LOG_NOTICE, "%08u, Landing...", clock->now());
                ER ercd = wup_tsk(MAIN_TASK); // wake up the main task
                assert(ercd == E_OK);
                
                // make sure this routine is NOT executed before it is killed by the main task
                clock->sleep(PERIOD_CAP_TSK/2); // wait a while
                state = ST_landing;
            }
            break;
        case ST_challenging:
            break;
        case ST_landing:
            break;
    }
}

void Captain::land() {
    if (activeNavigator != NULL) {
        activeNavigator->goOffDuty();
    }
    leftMotor->reset();
    rightMotor->reset();
    
    delete anchorWatch;
    delete lineTracer;
    delete seesawCrimber;
    delete limboDancer;
    delete harbourPilot;
    observer->goOffDuty();
    delete observer;
    // deregister cyclic handler from EV3RT
    ev3_stp_cyc(CYC_CAP_TSK);
    clock->sleep(2*PERIOD_CAP_TSK); // wait a while
}

Captain::~Captain() {
    _debug(syslog(LOG_NOTICE, "%08u, Captain destructor", clock->now()));
}
