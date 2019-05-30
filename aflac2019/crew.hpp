//
//  crew.hpp
//  cppTest
//
//  Created by Wataru Taniguchi on 2019/04/28.
//  Copyright © 2019 Wataru Taniguchi. All rights reserved.
//

#ifndef crew_hpp
#define crew_hpp

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
#define TAIL_ANGLE_STAND_UP  90  /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3  /* バランス走行時の角度[度] */
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */
//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START         '1'    /* リモートスタートコマンド */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

class Observer {
private:
    FILE*           bt;      /* Bluetoothファイルハンドル */
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;
    bool check_touch(void);
    bool check_sonar(void);
    bool check_bt(void);
    bool check_backButton(void);
protected:
public:
    Observer();
    Observer(TouchSensor* ts,SonarSensor* ss);
    void goOnDuty();
    void operate(); // method to invoke from the cyclic handler
    void goOffDuty();
    ~Observer();
};

class Navigator {
protected:
    int8_t forward;      /* 前後進命令 */
    int8_t turn;         /* 旋回命令 */
    int8_t pwm_L, pwm_R; /* 左右モータPWM出力 */
    Motor*      leftMotor;
    Motor*      rightMotor;
    Motor*      tailMotor;
    GyroSensor* gyroSensor;
    void cancelBacklash(int8_t lpwm, int8_t rpwm, int32_t *lenc, int32_t *renc);
    void controlTail(int32_t angle);
public:
    Navigator();
    virtual void goOnDuty() = 0;
    virtual void operate() = 0;
    virtual void goOffDuty() = 0;
    virtual ~Navigator();
};

class AnchorWatch : public Navigator {
private:
protected:
public:
    AnchorWatch();
    AnchorWatch(Motor* tm);
    void goOnDuty();
    void operate(); // method to invoke from the cyclic handler
    void goOffDuty();
    ~AnchorWatch();
};

class LineTracer : public Navigator {
private:
    ColorSensor*    colorSensor;
    int32_t motor_ang_l, motor_ang_r;
    int32_t gyro, volt;
protected:
public:
    LineTracer();
    LineTracer(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs);
    void goOnDuty();
    void operate(); // method to invoke from the cyclic handler
    void goOffDuty();
    ~LineTracer();
};

class SeesawCrimber : public Navigator {
protected:
public:
    SeesawCrimber();
    SeesawCrimber(int16_t*);
    void goOnDuty();
    void operate(); // method to invoke from the cyclic handler
    void goOffDuty();
    ~SeesawCrimber();
};

class LimboDancer : public Navigator {
protected:
public:
    LimboDancer();
    LimboDancer(int16_t*);
    void goOnDuty();
    void operate(); // method to invoke from the cyclic handler
    void goOffDuty();
    ~LimboDancer();
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
protected:
public:
    Captain();
    void takeoff();
    void operate(); // method to invoke from the cyclic handler
    void land();
    ~Captain();
};

#endif /* crew_hpp */
