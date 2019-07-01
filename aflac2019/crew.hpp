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
#define LIGHT_WHITE          60  /* 白色の光センサ値 */
#define LIGHT_BLACK           3  /* 黒色の光センサ値 */
#define HSV_V_WHITE         280
#define HSV_V_BLACK          10
#define HSV_V_BLUE           90
#define SONAR_ALERT_DISTANCE 30  /* 超音波センサによる障害物検知距離[cm] */
#define TAIL_ANGLE_STAND_UP  90  /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3  /* バランス走行時の角度[度] */
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */

#define TIRE_DIAMETER    100.0F  // diameter of tire in milimater
#define WHEEL_TREAD      175.0F  // distance between the right and left wheels

//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START_R     'R' // R-mode start command
#define CMD_START_r     'r' // R-mode start command
#define CMD_START_L     'L' // L-mode start command
#define CMD_START_l     'l' // L-mode start command
#define CMD_DANCE_D     'D'
#define CMD_DANCE_d     'd'
#define CMD_CRIMB_C     'C'
#define CMD_CRIMB_c     'c'

// machine state
#define ST_takingOff    1
#define ST_tracing_L    2
#define ST_dancing      3
#define ST_tracing_R    4
#define ST_crimbing     5
#define ST_stopping     6
#define ST_landing      7

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

#define PERIOD_TRACE_MSG    1000    /* Trace message in every 1000 ms */
#define M_2PI    (2.0 * M_PI)

typedef struct {
    uint16_t h; // Hue
    uint16_t s; // Saturation
    uint16_t v; // Value of brightness
} hsv_raw_t;

void rgb_to_hsv(rgb_raw_t rgb, hsv_raw_t& hsv);

class Radioman {
private:
    FILE*           bt;      /* Bluetoothファイルハンドル */
public:
    Radioman();
    void operate(); // method to invoke from the task handler
    ~Radioman();
};

class Observer {
private:
    Motor*          leftMotor;
    Motor*          rightMotor;
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;
    bool check_touch(void);
    bool check_sonar(void);
    bool check_backButton(void);
    double distance, azimuth, locX, locY;
    int32_t prevAngL, prevAngR;
protected:
public:
    Observer();
    Observer(Motor* lm, Motor* rm, TouchSensor* ts,SonarSensor* ss);
    void goOnDuty();
    void reset();
    int32_t getDistance();
    int16_t getAzimuth();
    int32_t getLocX();
    int32_t getLocY();
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
    int16_t math_limit(int16_t input, int16_t min, int16_t max);
    int16_t computePID(int16_t sensor, int16_t target);
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
