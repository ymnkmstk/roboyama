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

void rgb_to_hsv(rgb_raw_t rgb, hsv_raw_t& hsv) {
    uint16_t max, min, cr, cg, cb, h;
    
    max = rgb.r;
    if(max < rgb.g) max = rgb.g;
    if(max < rgb.b) max = rgb.b;
    
    min = rgb.r;
    if(min > rgb.g) min = rgb.g;
    if(min > rgb.b) min = rgb.b;
    
    hsv.v = max;
    
    if (!max) {
        hsv.s = 0;
        hsv.h = 0;
    } else {
        hsv.s = 255 * (max - min) / (double)max;
        cr = (max - rgb.r) / (double)(max - min);
        cg = (max - rgb.g) / (double)(max - min);
        cb = (max - rgb.b) / (double)(max - min);
        
        if (max == rgb.r) {
            h = cb - cg;
        } else if (max == rgb.g) {
            h = 2 + cr - cb;
        } else {
            h = 4 + cg - cr;
        }
        h *= 60;
        if (h < 0) h += 360;
        hsv.h = h;
    }
}

Radioman::Radioman() {
    _debug(syslog(LOG_NOTICE, "%08u, Radioman constructor", clock->now()));
    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);
}

void Radioman::operate() {
    uint8_t c = fgetc(bt); /* 受信 */
    fputc(c, bt); /* エコーバック */
    switch(c)
    {
        case CMD_START_R:
        case CMD_START_r:
            _debug(syslog(LOG_NOTICE, "%08u, StartCMD R-mode received", clock->now()));
            captain->decide(EVT_cmdStart_R);
            break;
        case CMD_START_L:
        case CMD_START_l:
            _debug(syslog(LOG_NOTICE, "%08u, StartCMD L-mode received", clock->now()));
            captain->decide(EVT_cmdStart_L);
            break;
        case CMD_DANCE_D:
        case CMD_DANCE_d:
            _debug(syslog(LOG_NOTICE, "%08u, LimboDancer forced by command", clock->now()));
            captain->decide(EVT_cmdDance);
            break;
        case CMD_CRIMB_C:
        case CMD_CRIMB_c:
            _debug(syslog(LOG_NOTICE, "%08u, SeesawCrimber forced by command", clock->now()));
            captain->decide(EVT_cmdCrimb);
            break;
        case CMD_PILOT_P:
        case CMD_PILOT_p:
            _debug(syslog(LOG_NOTICE, "%08u, HarbourPilot forced by command", clock->now()));
            captain->decide(EVT_cmdPilot);
            break;
        default:
            break;
    }
}

Radioman::~Radioman() {
    _debug(syslog(LOG_NOTICE, "%08u, Radioman destructor", clock->now()));
    fclose(bt);
}

Observer::Observer(Motor* lm, Motor* rm, TouchSensor* ts,SonarSensor* ss) {
    _debug(syslog(LOG_NOTICE, "%08u, Observer constructor", clock->now()));
    leftMotor   = lm;
    rightMotor  = rm;
    touchSensor = ts;
    sonarSensor = ss;
    distance = 0.0;
    azimuth = 0.0;
    locX = 0.0;
    locY = 0.0;
    prevAngL = 0;
    prevAngR = 0;
    touch_flag = false;
    sonar_flag = false;
    backButton_flag = false;
}

void Observer::goOnDuty() {
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

    if (check_touch() && !touch_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, TouchSensor flipped on", clock->now()));
        touch_flag = true;
        captain->decide(EVT_touch_On);
    } else if (!check_touch() && touch_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, TouchSensor flipped off", clock->now()));
        touch_flag = false;
        captain->decide(EVT_touch_Off);
    }
    if (check_sonar() && !sonar_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, SonarSensor flipped on", clock->now()));
        sonar_flag = true;
        captain->decide(EVT_sonar_On);
    } else if (!check_sonar() && sonar_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, SonarSensor flipped off", clock->now()));
        sonar_flag = false;
        captain->decide(EVT_sonar_Off);
   }
    if (check_backButton() && !backButton_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, Back button flipped on", clock->now()));
        backButton_flag = true;
        captain->decide(EVT_backButton_On);
    } else if (!check_backButton() && backButton_flag) {
        _debug(syslog(LOG_NOTICE, "%08u, Back button flipped off", clock->now()));
        backButton_flag = false;
        captain->decide(EVT_backButton_Off);
    }
}

void Observer::goOffDuty() {
    // deregister cyclic handler from EV3RT
    ev3_stp_cyc(CYC_OBS_TSK);
    clock->sleep(PERIOD_OBS_TSK/2); // wait a while
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

int16_t Navigator::math_limit(int16_t input, int16_t min, int16_t max) {
    if (input < min) {
        return min;
    } else if (input > max) {
        return max;
    }
    return input;
}

int16_t Navigator::computePID(int16_t sensor, int16_t target) {
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
    frozen      = false;
}

void LineTracer::haveControl() {
    activeNavigator = this;
    syslog(LOG_NOTICE, "%08u, LineTracer has control", clock->now());
}

void LineTracer::operate() {
    controlTail(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */

    if (frozen) {
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
        // PID control by brightness
        int16_t sensor = colorSensor->getBrightness();
        int16_t target = (LIGHT_WHITE + LIGHT_BLACK)/2;
        /*
        // PID control by V in HSV
        rgb_raw_t cur_rgb;
        hsv_raw_t cur_hsv;
        colorSensor->getRawColor(cur_rgb);
        rgb_to_hsv(cur_rgb, cur_hsv);
        int16_t sensor = cur_hsv.v;
        int16_t target = (HSV_V_BLACK + HSV_V_WHITE)/2;
        */
        
        if (state == ST_tracing_L || state == ST_dancing) {
            turn = computePID(sensor, target);
        } else {
            // state == ST_tracing_R || state == ST_crimbing
            turn = (-1) * computePID(sensor, target);
        }
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
        //_debug(syslog(LOG_NOTICE, "%08u, LineTracer::operate(): pwm_L = %d, pwm_R = %d, distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), pwm_L, pwm_R, observer->getDistance(), observer->getAzimuth(), observer->getLocX(), observer->getLocY()));
        _debug(syslog(LOG_NOTICE, "%08u, LineTracer::operate(): distance = %d, azimuth = %d, x = %d, y = %d", clock->now(), observer->getDistance(), observer->getAzimuth(), observer->getLocX(), observer->getLocY()));
    }
}

void LineTracer::freeze() {
    frozen = true;
}

void LineTracer::unfreeze() {
    frozen = false;
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
    
    observer = new Observer(leftMotor, rightMotor, touchSensor, sonarSensor);
    observer->goOnDuty();
    limboDancer = new LimboDancer(leftMotor, rightMotor, tailMotor, gyroSensor, colorSensor);
    seesawCrimber = new SeesawCrimber(leftMotor, rightMotor, tailMotor, gyroSensor, colorSensor);
    lineTracer = new LineTracer(leftMotor, rightMotor, tailMotor, gyroSensor, colorSensor);
    harbourPilot = new HarbourPilot(leftMotor, rightMotor, tailMotor, gyroSensor, colorSensor);
    
    /* 尻尾モーターのリセット */
    tailMotor->reset();
    
    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    state = ST_takingOff;
    anchorWatch = new AnchorWatch(tailMotor);
    anchorWatch->goOnDuty();
    anchorWatch->haveControl();

    act_tsk(RADIO_TASK);
}

void Captain::decide(uint8_t event) {
    switch (state) {
        case ST_takingOff:
            switch (event) {
                case EVT_cmdStart_R:
                case EVT_cmdStart_L:
                case EVT_touch_On:
                    if (event == EVT_cmdStart_R) {
                        state = ST_tracing_R;
                    } else {
                        state = ST_tracing_L;
                    }
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
                    break;
                default:
                    break;
            }
            break;
        case ST_tracing_R:
        case ST_tracing_L:
            switch (event) {
                case EVT_backButton_On:
                    state = ST_landing;
                    syslog(LOG_NOTICE, "%08u, Landing...", clock->now());
                    { // complile fails without this paren
                    ER ercd = wup_tsk(MAIN_TASK); // wake up the main task
                    assert(ercd == E_OK);
                    }
                    break;
                case EVT_sonar_On:
                    lineTracer->freeze();
                    break;
                case EVT_sonar_Off:
                    lineTracer->unfreeze();
                    break;
                case EVT_cmdDance:
                    state = ST_dancing;
                    limboDancer->haveControl();
                    break;
                case EVT_cmdCrimb:
                    state = ST_crimbing;
                    seesawCrimber->haveControl();
                    break;
                case EVT_cmdPilot:
                    state = ST_stopping;
                    harbourPilot->haveControl();
                    break;
                default:
                    break;
            }
            break;
        case ST_dancing:
            break;
        case ST_crimbing:
            break;
        case ST_stopping:
            break;
        case ST_landing:
            break;
        default:
            break;
    }
}

void Captain::land() {
    ter_tsk(RADIO_TASK);

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
}

Captain::~Captain() {
    _debug(syslog(LOG_NOTICE, "%08u, Captain destructor", clock->now()));
}
