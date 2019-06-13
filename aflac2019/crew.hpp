//
//  crew.hpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/04/28.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef crew_hpp
#define crew_hpp

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

#include <cinttypes>
#include <cmath>
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
using namespace ev3api;

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define GYRO_OFFSET           0  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE          55  /* 白色の光センサ値 */
#define LIGHT_BLACK           0  /* 黒色の光センサ値 */
#define SONAR_ALERT_DISTANCE 30  /* 超音波センサによる障害物検知距離[cm] */
#define TAIL_ANGLE_STAND_UP  85  /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3  /* バランス走行時の角度[度] */
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */

#define TIRE_DIAMETER    100.0F  // diameter of tire in milimater
#define WHEEL_TREAD      175.0F  // distance between the right and left wheels

//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START         '1'    /* リモートスタートコマンド */

// machine state
#define ST_takingOff    1
#define ST_tracing      2
#define ST_challenging  3
#define ST_landing      4

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

#define PERIOD_TRACE_MSG    1000    /* Trace message in every 1000 ms */

class Observer {
private:
    FILE*           bt;      /* Bluetoothファイルハンドル */
    Motor*          leftMotor;
    Motor*          rightMotor;
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;
    bool check_touch(void);
    bool check_sonar(void);
    bool check_bt(void);
    bool check_backButton(void);
    float distance, azimuth, locX, locY;
    int32_t prevAngL, prevAngR;
protected:
public:
    Observer();
    Observer(Motor* lm, Motor* rm, TouchSensor* ts,SonarSensor* ss);
    void goOnDuty();
    void reset();
    float getDistance();
    float getAzimuth();
    float getLocX();
    float getLocY();
    void operate(); // method to invoke from the cyclic handler
    void goOffDuty();
    ~Observer();
};

class Navigator {
private:
    long double kp, ki, kd;   /* PID constant */
    int16_t diff[2];
    long double integral;
protected:
    int8_t forward;      /* 前後進命令 */
    int8_t turn;         /* 旋回命令 */
    int8_t pwm_L, pwm_R; /* 左右モータPWM出力 */
    int16_t         trace_pwmT, trace_pwmLR;
    Motor*          leftMotor;
    Motor*          rightMotor;
    Motor*          tailMotor;
    GyroSensor*     gyroSensor;
    ColorSensor*    colorSensor;
    void cancelBacklash(int8_t lpwm, int8_t rpwm, int32_t *lenc, int32_t *renc);
    void controlTail(int32_t angle);
    void setPIDconst(long double p, long double i, long double d);
    int8_t math_limit(int8_t input, int8_t min, int8_t max);
    int8_t computePID(int8_t sensor, int8_t target);
public:
    Navigator();
    void goOnDuty();
    virtual void haveControl() = 0;
    virtual void operate() = 0;
    void goOffDuty();
    virtual ~Navigator();
};

class AnchorWatch : public Navigator {
private:
protected:
public:
    AnchorWatch();
    AnchorWatch(Motor* tm);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~AnchorWatch();
};

class LineTracer : public Navigator {
private:
    int32_t motor_ang_l, motor_ang_r;
    int32_t gyro, volt;
protected:
public:
    LineTracer();
    LineTracer(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~LineTracer();
};

#include "SeesawCrimber.hpp"
#include "LimboDancer.hpp"

class HarbourPilot : public Navigator {
protected:
public:
    HarbourPilot();
    HarbourPilot(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~HarbourPilot();
};

class Captain {
private:
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;
    ColorSensor*    colorSensor;
    GyroSensor*     gyroSensor;
    Motor*          leftMotor;
    Motor*          rightMotor;
    Motor*          tailMotor;
    AnchorWatch*    anchorWatch;
    LineTracer*     lineTracer;
    SeesawCrimber*  seesawCrimber;
    LimboDancer*    limboDancer;
    HarbourPilot*   harbourPilot;
protected:
public:
    Captain();
    void takeoff();
    void operate(); // method to invoke from the cyclic handler
    void land();
    ~Captain();
};

extern Observer*    observer;
extern Navigator*   activeNavigator;
extern Clock*       clock;
extern uint8_t      state;

#endif /* crew_hpp */
