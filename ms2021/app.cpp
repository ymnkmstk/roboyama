/*
    app.cpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#include "app.h"
#include "appusr.hpp"

/* this is to avoid linker error, undefined reference to `__sync_synchronize' */
extern "C" void __sync_synchronize() {}

/* global variables */
FILE*           bt;
Clock*          clock;
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
FilteredColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
SRLF*           srlf_l;
FilteredMotor*  leftMotor;
SRLF*           srlf_r;
FilteredMotor*  rightMotor;
Motor*          tailMotor;
Motor*          armMotor;
Plotter*        plotter;
Logger*         logger;

BrainTree::BehaviorTree* tr_calibration = nullptr;
BrainTree::BehaviorTree* tr_run         = nullptr;
BrainTree::BehaviorTree* tr_slalom      = nullptr;
BrainTree::BehaviorTree* tr_garage      = nullptr;
State state = ST_initial;

class IsTouchOn : public BrainTree::Node {
public:
    Status update() override {
        /* keep resetting clock until touch sensor gets pressed */
        clock->reset();
        if (touchSensor->isPressed()) {
            _log("touch sensor pressed.");
            /* indicate departure by LED color */
            ev3_led_set_color(LED_GREEN);
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
};

class IsBackOn : public BrainTree::Node {
public:
    Status update() override {
        if (ev3_button_is_pressed(BACK_BUTTON)) {
            _log("back button pressed.");
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
};

/* colorCode = 1:Black 2:Blue 3:Red 4:Yellow 5:Green 6:White */
class IsTargetColorDetected : public BrainTree::Node {
public:
    IsTargetColorDetected(int c) : colorCode(c) {}
    Status update() override {

        rgb_raw_t cur_rgb;
        colorSensor->getRawColor(cur_rgb);
        switch(colorCode){
            case COLOR_BLACK:
                if (cur_rgb.r <=50 && cur_rgb.g <=45 && cur_rgb.b <=60) {
                    _log("found black.");
                    return Status::Success;
                }
                break;
            case COLOR_BLUE:
                if (cur_rgb.b - cur_rgb.r > 45 && cur_rgb.b <= 255 && cur_rgb.r <= 255) {
                    _log("found blue.");
                    return Status::Success;
                }
                break;
            case COLOR_RED:
                if (cur_rgb.r - cur_rgb.b >= 40 && cur_rgb.g < 60 && cur_rgb.r - cur_rgb.g > 30) {
                    _log("found red.");
                    return Status::Success;
                }
                break;
            case COLOR_YELLOW:
                if (cur_rgb.r + cur_rgb.g - cur_rgb.b >= 130 &&  cur_rgb.r - cur_rgb.g <= 30) {
                    _log("found Yellow.");
                    return Status::Success;
                }
                break;
            case COLOR_GREEN:
                if (cur_rgb.r <= 13 && cur_rgb.b <= 50 && cur_rgb.g > 60) {
                    _log("found Green.");
                    return Status::Success;
                }
                break;
            case COLOR_WHITE:
                if (cur_rgb.r > 100 && cur_rgb.b > 100 && cur_rgb.g > 100) {
                    _log("found White.");
                    return Status::Success;
                }
                break;
            default:
                break;
        }
        return Status::Running;
    }
protected:
    int8_t colorCode;
};

class IsSonarOn : public BrainTree::Node {
public:
    IsSonarOn(int32_t d) : alertDistance(d) {}
    Status update() override {
        int32_t distance = sonarSensor->getDistance();
        if ((distance <= alertDistance) && (distance >= 0)) {
            _log("sonar alert at %d", distance);
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
protected:
    int32_t alertDistance;
};

class IsDistanceEarned : public BrainTree::Node {
public:
    IsDistanceEarned(int32_t d) : deltaDistTarget(d),updated(false),earned(false) {}
    Status update() override {
        if (!updated) {
            originalDist = plotter->getDistance();
            updated = true;
        }
        int32_t deltaDist = plotter->getDistance() - originalDist;
        
        if(deltaDist< 0){deltaDist= deltaDist* (-1);}

        if (deltaDist >= deltaDistTarget) {
            if (!earned) {
                _log("Delta %d is earned at absolute distance %d.", deltaDistTarget, plotter->getDistance());
                earned = true;
            }
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
protected:
    int32_t deltaDistTarget, originalDist;
    bool updated, earned;
};

/* argument -> 100 = 1 sec */
class IsTimeEarned : public BrainTree::Node {
public:
    IsTimeEarned(int32_t t) : deltaTimeTarget(t),updated(false),earned(false),flg(false) {}
     Status update() override {
        if (!updated) {
            originalTime = round((int32_t)clock->now()/10000);
            updated = true;
        }
        deltaTime = round((int32_t)clock->now() / 10000) - originalTime;

        if (deltaTime >= deltaTimeTarget ) {
            if (!earned) {
                 _log("Delta %d getnow= %d", deltaTime,clock->now());
                earned = true;
            }
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
protected:
    int32_t deltaTimeTarget, originalTime, deltaTime;
    bool updated, earned, flg;
};

class TraceLine : public BrainTree::Node {
public:
    TraceLine(int s, int t, double p, double i, double d) : speed(s),target(t),prevAngL(0),prevAngR(0) {
        ltPid = new PIDcalculator(p, i, d, PERIOD_UPD_TSK, -speed, speed);
    }
    ~TraceLine() {
        delete ltPid;
    }
    Status update() override {
        int16_t sensor;
        int8_t forward, turn, pwm_L, pwm_R;
        rgb_raw_t cur_rgb;

        colorSensor->getRawColor(cur_rgb);
        sensor = cur_rgb.r;
        /* compute necessary amount of steering by PID control */
        turn = (-1) * _COURSE * ltPid->compute(sensor, (int16_t)target);
        forward = speed;
        /* steer EV3 by setting different speed to the motors */
        pwm_L = forward - turn;
        pwm_R = forward + turn;
        srlf_l->setRate(0);
        leftMotor->setPWM(pwm_L);
        srlf_r->setRate(0);
        rightMotor->setPWM(pwm_R);
        return Status::Running;
    }
protected:
    int speed, target;
    PIDcalculator* ltPid;
    int32_t prevAngL, prevAngR;
};

/*  usage:
    ".leaf<RunAsInstructed>(pwm_l, pwm_r, trpz_calc_flg, updown_interval, updown_pwm)"
    is to move the robot at the instructed speed.
    trpz_calc_flg enables a trapezoidal control of the motors until the current speed gradually reaches the instructed target speed. */
class RunAsInstructed : public BrainTree::Node {
public:
    RunAsInstructed(int pwm_l, int pwm_r, bool trpz_calc_flg, int updown_interval, int updown_pwm) : pwmL(pwm_l),pwmR(pwm_r),trpzCalcFlg(trpz_calc_flg) {
        /* updown_interval == 0 means updown_interval == C_INTERVAL */
        if (updown_interval == 0) {
            updownInterval = C_INTERVAL;
        } else {
            updownInterval = updown_interval;
        }
        /* updown_pwm == 0 means updown_interval == C_PWD */
        if (updown_pwm == 0) {
            updownPwm = C_PWD;
        } else {
            updownPwm = updown_pwm;
        }
    }
    Status update() override {
        if(!trpzCalcFlg){
            leftMotor->setPWM(pwmL);
            rightMotor->setPWM(pwmR);
        }else{
            double srewRate = (double)updownPwm/(double)updownInterval;
            srlf_l->setRate(srewRate);
            srlf_r->setRate(srewRate);
            leftMotor->setPWM(pwmL);
            rightMotor->setPWM(pwmR);
        }
        return Status::Running;
    }
protected:
    int pwmL, pwmR, updownInterval, updownPwm;
    bool trpzCalcFlg;
};

/*

    usage:
    ".leaf<RunAsInstructed>(pwm_l, pwm_r, srew_rate)"
    is to move the robot at the instructed speed.
    srew_rate = 0.0 indidates NO tropezoidal motion.
    srew_rate = 0.5 instructs FilteredMotor to change 1 pwm every two executions of update()
    until the current speed gradually reaches the instructed target speed.

class RunAsInstructed : public BrainTree::Node {
public:
    RunAsInstructed(int pwm_l, int pwm_r, double srew_rate) : pwmL(pwm_l),pwmR(pwm_r),srewRate(srew_rate) {}
    Status update() override {
        srlf_l->setRate(srewRate);
        srlf_r->setRate(srewRate);
        leftMotor->setPWM(pwmL);
        rightMotor->setPWM(pwmR);
        return Status::Running;
    }
protected:
    int pwmL, pwmR;
    double srewRate;
};
*/

class ShiftArmPosition : public BrainTree::Node {
public:
    ShiftArmPosition(int armpwm) : armPwm(armpwm),traceCnt(0) {}
    Status update() override {
        armMotor->setPWM(armPwm);
        return Status::Running;
    }
protected:
    int armPwm;
private:
    int traceCnt;
};

/*  usage:
    ".leaf<MoveToLine>(speed, target)"
    is to move robot straight ahead till color sensor value reaches to the target at the speed  */
class MoveToLine : public BrainTree::Node {
public:
    MoveToLine(int s, int t) : speed(s),target(t) {}
    Status update() override {
        int16_t sensor;
        rgb_raw_t cur_rgb;

        colorSensor->getRawColor(cur_rgb);
        sensor = cur_rgb.r;

        if (sensor >= target) {
            /* move EV3 closer to the line */
            leftMotor->setPWM(speed);
            rightMotor->setPWM(speed);
            /* display trace message in every PERIOD_TRACE_MSG ms */
            if (++traceCnt * PERIOD_UPD_TSK >= PERIOD_TRACE_MSG) {
                traceCnt = 0;
                int32_t angL = plotter->getAngL();
                int32_t angR = plotter->getAngR();
                _log("sensor = %d, deltaAngDiff = %d, locX = %d, locY = %d, degree = %d, distance = %d",
                    sensor, (int)((angL-prevAngL)-(angR-prevAngR)),
                    (int)plotter->getLocX(), (int)plotter->getLocY(),
                    (int)plotter->getDegree(), (int)plotter->getDistance());
                prevAngL = angL;
                prevAngR = angR;
            }
            return Status::Running;
        } else {
            return Status::Success;
        }
    }
protected:
    int speed, target;
    int32_t prevAngL, prevAngR;
private:
    int traceCnt;
};

/*  usage:
    ".leaf<RotateEV3>(30 * _COURSE, speed, trpz_calc_flg)"
    is to rotate robot 30 degrees clockwise at the speed when in L course */
class RotateEV3 : public BrainTree::Node {
public:
    RotateEV3(int16_t degree, int s, bool trpz_calc_flg) : deltaDegreeTarget(degree),speed(s), trpzCalcFlg(trpz_calc_flg),updated(false) {
        deltaDegreeTrpzMtrCtrl = 0;
        assert(degree >= -180 && degree <= 180);
        if (degree > 0) {
            clockwise = 1;
        } else {
            clockwise = -1;
        }
    }
    Status update() override {
        if (!updated) {
            originalDegree = plotter->getDegree();
            updated = true;
        }
        int16_t deltaDegree = plotter->getDegree() - originalDegree;
        if (deltaDegree > 180) {
            deltaDegree -= 360;
        } else if (deltaDegree < -180) {
            deltaDegree += 360;
        }

        if (clockwise * deltaDegree < clockwise * deltaDegreeTarget) {

            if(!trpzCalcFlg){
                leftMotor->setPWM(clockwise * speed);
                rightMotor->setPWM((-clockwise) * speed);
            }else{
                srlf_l->setRate(1.0);
                srlf_r->setRate(1.0);
                if(clockwise * speed <= leftMotor->getPWM() && clockwise * deltaDegree < floor(clockwise * deltaDegreeTarget * 0.5) && deltaDegreeTrpzMtrCtrl == 0){
                    deltaDegreeTrpzMtrCtrl = deltaDegree; 
                }else if(clockwise * speed > leftMotor->getPWM() && clockwise * deltaDegree >= floor(clockwise * deltaDegreeTarget * 0.5) && deltaDegreeTrpzMtrCtrl == 0){
                    deltaDegreeTrpzMtrCtrl = deltaDegreeTarget;
                }

                if(clockwise * deltaDegree < clockwise * deltaDegreeTarget - deltaDegreeTrpzMtrCtrl ){
                    leftMotor->setPWM(clockwise * speed);
                    rightMotor->setPWM((-clockwise) * speed);
                }else{
                    leftMotor->setPWM(clockwise * 3);
                    rightMotor->setPWM((-clockwise) * 3);
                }
            }
            return Status::Running;
            
        } else {
            return Status::Success;
        }
    }
private:
    int16_t deltaDegreeTarget, originalDegree, deltaDegreeTrpzMtrCtrl;
    int clockwise, speed;
    bool updated, trpzCalcFlg;
};

class ClimbBoard : public BrainTree::Node { 
public:
    ClimbBoard(int direction, int count) : dir(direction), cnt(count) {}
    Status update() override {
        curAngle = gyroSensor->getAngle();
            if(cnt >= 1){
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                armMotor->setPWM(-50);
                cnt++;
                if(cnt >= 150){
                    return Status::Success;
                }
                return Status::Running;
            }else{
                armMotor->setPWM(80);
                leftMotor->setPWM(23);
                rightMotor->setPWM(23);
                
                if(curAngle < -9){
                    prevAngle = curAngle;
                }
                if (prevAngle < -9 && curAngle >= 0){
                    ++cnt;
                    _log("ON BOARD");
                }
                return Status::Running;
            }
    }
private:
    int8_t dir;
    int cnt;
    int32_t curAngle;
    int32_t prevAngle;
};

/* a cyclic handler to activate a task */
void task_activator(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK || E_QOVR);
    if (ercd != E_OK) {
        syslog(LOG_NOTICE, "act_tsk() returned %d", ercd);
    }
}

void main_task(intptr_t unused) {
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);
    /* create and initialize EV3 objects */
    clock       = new Clock();
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_2);
    colorSensor = new FilteredColorSensor(PORT_3);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new FilteredMotor(PORT_C);
    rightMotor  = new FilteredMotor(PORT_B);
    tailMotor   = new Motor(PORT_D);
    armMotor    = new Motor(PORT_A);
    plotter     = new Plotter(leftMotor, rightMotor, gyroSensor);
    logger      = new Logger(plotter,leftMotor, rightMotor, gyroSensor,colorSensor,sonarSensor,clock);

    /* FIR parameters for a low-pass filter with normalized cut-off frequency of 0.2
        using a function of the Hamming Window */
    const int FIR_ORDER = 4; 
    const double hn[FIR_ORDER+1] = { 7.483914270309116e-03, 1.634745733863819e-01, 4.000000000000000e-01, 1.634745733863819e-01, 7.483914270309116e-03 };
    /* set filters to FilteredColorSensor */
    Filter *lpf_r = new FIR_Transposed(hn, FIR_ORDER);
    Filter *lpf_g = new FIR_Transposed(hn, FIR_ORDER);
    Filter *lpf_b = new FIR_Transposed(hn, FIR_ORDER);
    colorSensor->setRawColorFilters(lpf_r, lpf_g, lpf_b);

    srlf_l = new SRLF(0.0);
    leftMotor->setPWMFilter(srlf_l);
    srlf_r = new SRLF(0.0);
    rightMotor->setPWMFilter(srlf_r);

    /* BEHAVIOR TREE DEFINITION */

    /* robot starts when touch sensor is turned on */
    tr_calibration = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .decorator<BrainTree::UntilSuccess>()
            .leaf<IsTouchOn>()
        .end()
        .build();

    /* robot continues running unless:
        ultrasonic sonar detects an obstacle or
        back button is pressed or
        the second blue part of line is reached at further than BLUE_DISTANCE */
    tr_run = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::ParallelSequence>(1,4)
            //.leaf<IsSonarOn>(SONAR_ALERT_DISTANCE)
            .leaf<IsBackOn>()
            .composite<BrainTree::ParallelSequence>(2,2)
                .leaf<IsDistanceEarned>(BLUE_DISTANCE)
                .composite<BrainTree::MemSequence>()
                    .leaf<IsTargetColorDetected>(COLOR_BLUE)
                    .leaf<IsTargetColorDetected>(COLOR_BLACK)
                    .leaf<IsTargetColorDetected>(COLOR_BLUE)
                .end()
            .end()
            .composite<BrainTree::MemSequence>()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(300)
                    //.leaf<TraceLine>(SPEED_NORM, GS_TARGET, P_CONST, I_CONST, D_CONST)
                    .leaf<TraceLine>(65, GS_TARGET, 0.75, 1.0, D_CONST)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(1450) // 1250
                    .leaf<TraceLine>(SPEED_FAST, GS_TARGET, P_CONST_FAST, I_CONST_FAST, D_CONST_FAST)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(1030) // 1250
                    .leaf<TraceLine>(65, GS_TARGET, 0.75, 1.0, D_CONST)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(130)
                    .leaf<RunAsInstructed>(65,65, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(450)
                    .leaf<RunAsInstructed>(37,100, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(400)
                    .leaf<RunAsInstructed>(100,92, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(290)
                    .leaf<RunAsInstructed>(100,44, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(155)
                    .leaf<RunAsInstructed>(100,100, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(410)
                    .leaf<RunAsInstructed>(100,52, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(575)
                    .leaf<RunAsInstructed>(63,100, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(280)
                    .leaf<RunAsInstructed>(44,100, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(240)
                    .leaf<RunAsInstructed>(100,75, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(390)
                    .leaf<RunAsInstructed>(100,37, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsTargetColorDetected>(COLOR_BLACK)
                    .leaf<RunAsInstructed>(100,90, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(50)
                    .leaf<RunAsInstructed>(70,70, true, 0, 2)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(300)
                    .leaf<TraceLine>(SPEED_NORM, GS_TARGET, P_CONST, I_CONST, D_CONST)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(1400)
                    .leaf<TraceLine>(SPEED_FAST, GS_TARGET, P_CONST_FAST, I_CONST_FAST, D_CONST_FAST)
                .end()
                .composite<BrainTree::ParallelSequence>(1,2)
                    .leaf<IsDistanceEarned>(3000)
                    .leaf<TraceLine>(65, GS_TARGET, 0.75, 1.0, D_CONST)
                .end()
            .end()
        .end()
        .build();

    tr_slalom = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::MemSequence>()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsDistanceEarned>(200)
                .leaf<RunAsInstructed>(35,35, true, 0, 2)
            .end()
            .leaf<ClimbBoard>(_COURSE, 0)
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(500)
                .leaf<TraceLine>(SPEED_SLOW, GS_TARGET_SLOW, P_CONST_SLOW, I_CONST_SLOW, D_CONST_SLOW)
            .end()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(300)
                .leaf<RunAsInstructed>(8,10, false, 0, 0)
            .end()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(1160)
                .leaf<TraceLine>(SPEED_SLOW, GS_TARGET_SLOW, P_CONST_SLOW, I_CONST_SLOW, D_CONST_SLOW)
            .end()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(800)
                .leaf<RunAsInstructed>(10,4, true, 2, 0)
            .end()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(595)
                .leaf<RunAsInstructed>(10,2, true, 2, 0)
            .end()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(285)
                .leaf<RunAsInstructed>(10,3, true, 2, 0)
            .end()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(285)
                .leaf<RunAsInstructed>(3,10, true, 2, 0)
            .end()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(305)
                .leaf<RunAsInstructed>(2,10, true, 2, 0)
            .end()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(350)
                .leaf<RunAsInstructed>(10,10, true, 2, 0)
            .end()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(420)
                .leaf<ShiftArmPosition>(30)
            .end()
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(100)
                .leaf<ShiftArmPosition>(-100)
                .leaf<RunAsInstructed>(0, 0, false, 0, 0)
            .end()
        .end()
        .build();

    tr_garage = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::MemSequence>()
            //スラロームから離れるため進みながらやや右に曲がる
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(170)
                .leaf<RunAsInstructed>(30,5, false, 0, 0)
            .end()
             //指定距離走行
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(40)
                .leaf<RunAsInstructed>(30,20, false, 0, 0)
            .end()
             //指定距離走行
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(80)
                .leaf<RunAsInstructed>(30,30, false, 0, 0)
            .end()
            //黒色検知
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTargetColorDetected>(COLOR_BLACK)
                .leaf<RunAsInstructed>(30,30, false, 0, 0)
            .end()
             //指定距離走行
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(120)
                .leaf<RunAsInstructed>(30,30, false, 0, 0)
            .end()
            //黒色検知
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTargetColorDetected>(COLOR_BLACK)
                .leaf<RunAsInstructed>(30,30, false, 0, 0)
            .end()
             //指定距離走行
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(140)
                .leaf<RunAsInstructed>(30,30, false, 0, 0)
            .end()
//             //左にXX度回転 10の速さ　台形駆動無
            .leaf<RotateEV3>(-80,10,false)
            // .leaf<RotateEV3>(-83,10,false)
             //指定距離走行
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(100)
                .leaf<RunAsInstructed>(30,30, false, 0, 0)
            .end()
            //黒色検知
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTargetColorDetected>(COLOR_BLACK)
                .leaf<RunAsInstructed>(30,30, false, 0, 0)
            .end()
            //左にXX度回転 10の速さ　台形駆動無
            .leaf<RotateEV3>(-75,10,false)
            //指定時間走行
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(5)
                .leaf<RunAsInstructed>(5,5, false, 0, 0)
            .end()
            //ライントレース
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(30)
                .leaf<TraceLine>(SPEED_SLOW, GS_TARGET_SLOW, P_CONST_SLOW, I_CONST_SLOW, D_CONST_SLOW)
            .end()
            // //赤色検知
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTargetColorDetected>(COLOR_RED)
                .leaf<RunAsInstructed>(30,30, false, 0, 0)
            .end()
            //指定時間走行
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(110)
                .leaf<RunAsInstructed>(30,30, false, 0, 0)
            .end()
            // //ライントレース
            // .composite<BrainTree::ParallelSequence>(1,2)
            //     .leaf<IsTimeEarned>(30)
            //     .leaf<TraceLine>(SPEED_SLOW, GS_TARGET_SLOW, P_CONST_SLOW, I_CONST_SLOW, D_CONST_SLOW)
            // .end()
            //やや左に指定時間走行
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(70)
                .leaf<RunAsInstructed>(37,50, false, 0, 0)
            .end()
            //指定時間走行
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(320)
                .leaf<RunAsInstructed>(50,50, false, 0, 0)
            .end()
            //やや右に指定時間走行
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(30)
                .leaf<RunAsInstructed>(50,40, false, 0, 0)
            .end()
            //青か黒色検知
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTargetColorDetected>(COLOR_BLUE)
                .leaf<IsTargetColorDetected>(COLOR_BLACK)
                .leaf<RunAsInstructed>(20,20, false, 0, 0)
            .end()
            //右回転
            .leaf<RotateEV3>(81,5,false)
            //まっすぐ距離調整用
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(10)
                .leaf<RunAsInstructed>(8,8, false, 0, 0)
            .end()
            //ライントレース
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(35)
               .leaf<TraceLine>(SPEED_SLOW, GS_TARGET_SLOW, P_CONST_SLOW, I_CONST_SLOW, D_CONST_SLOW)
            .end() 
            //まっすぐ距離調整用
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsTimeEarned>(235)
                .leaf<RunAsInstructed>(30,30, false, 0, 0)
            .end()
        .end()
        .build();

    /* register cyclic handler to EV3RT */
    sta_cyc(CYC_UPD_TSK);

    /* indicate initialization completion by LED color */
    _log("initialization completed.");
    ev3_led_set_color(LED_ORANGE);
    state = ST_calibrating;

    /* sleep until being waken up */
    _log("going to sleep...");
    ER ercd = slp_tsk();
    assert(ercd == E_OK);
    if (ercd != E_OK) {
        syslog(LOG_NOTICE, "slp_tsk() returned %d", ercd);
    }

    /* deregister cyclic handler from EV3RT */
    stp_cyc(CYC_UPD_TSK);
    /* destroy behavior tree */
    delete tr_garage;
    delete tr_slalom;
    delete tr_run;
    delete tr_calibration;
    /* destroy EV3 objects */
    delete lpf_b;
    delete lpf_g;
    delete lpf_r;
    delete plotter;
    delete armMotor;
    delete tailMotor;
    delete rightMotor;
    delete leftMotor;
    delete gyroSensor;
    delete colorSensor;
    delete sonarSensor;
    delete touchSensor;
    delete clock;
    _log("being terminated...");
    fclose(bt);
    ETRoboc_notifyCompletedToSimulator();
    ext_tsk();
}

/* periodic task to update the behavior tree */
void update_task(intptr_t unused) {
    BrainTree::Node::Status status;
    ER ercd;

    colorSensor->sense();
    plotter->plot();
    switch (state) {
    case ST_calibrating:
        if (tr_calibration != nullptr) {
            status = tr_calibration->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                switch (JUMP) { /* JUMP = 1 or 2 is for testing only */
                    case 1:
                        state = ST_slalom;
                        _log("State changed: ST_calibration to ST_slalom");
                        break;
                    case 2:
                        state = ST_garage;
                        _log("State changed: ST_calibration to ST_garage");
                        break;
                    default:
                        state = ST_running;
                        _log("State changed: ST_calibration to ST_running");
                        break;
                }
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_ending;
                _log("State changed: ST_calibration to ST_ending");
                break;
            default:
                break;
            }
        }
        break;
    case ST_running:
        if (tr_run != nullptr) {
            status = tr_run->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                state = ST_slalom;
                _log("State changed: ST_running to ST_slalom");
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_ending;
                _log("State changed: ST_running to ST_ending");
                break;
            default:
                break;
            }
        }
        break;
    case ST_slalom:
        if (tr_slalom != nullptr) {
            status = tr_slalom->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
                state = ST_garage;
                _log("State changed: ST_slalom to ST_garage");
                break;
            case BrainTree::Node::Status::Failure:
                state = ST_ending;
                _log("State changed: ST_slalom to ST_ending");
                break;
            default:
                break;
            }
        }
        break;
    case ST_garage:
        if (tr_garage != nullptr) {
            status = tr_garage->update();
            switch (status) {
            case BrainTree::Node::Status::Success:
            case BrainTree::Node::Status::Failure:
                state = ST_ending;
                _log("State changed: ST_garage to ST_ending");
                break;
            default:
                break;
            }
        }
        break;
    case ST_ending:
        _log("waking up main...");
        /* wake up the main task */
        ercd = wup_tsk(MAIN_TASK);
        assert(ercd == E_OK);
        if (ercd != E_OK) {
            syslog(LOG_NOTICE, "wup_tsk() returned %d", ercd);
        }
        state = ST_end;
        _log("State changed: ST_ending to ST_end");
        break;    
    case ST_initial:
    case ST_end:
    default:
        break;
    }
    rightMotor->drive();
    leftMotor->drive();

    //Log output
    logger->outputLog(false,false,false,1,state);
}