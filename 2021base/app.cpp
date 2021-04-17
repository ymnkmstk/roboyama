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
FilteredColorSensor* filteredColorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Motor*          armMotor;
Plotter*        plotter;

BrainTree::BehaviorTree* tree = nullptr;
BrainTree::BehaviorTree* tree_test = nullptr; //sano family add

class IsTouchOn : public BrainTree::Node {
public:
    Status update() override {
        if (touchSensor->isPressed()) {
            _log("touch sensor pressed.");
            /* indicate departure by LED color */
            ev3_led_set_color(LED_GREEN);
            return Node::Status::Success;
        } else {
            return Node::Status::Failure;
        }
    }
};

class IsBackOn : public BrainTree::Node {
public:
    Status update() override {
        if (ev3_button_is_pressed(BACK_BUTTON)) {
            _log("back button pressed.");
            return Node::Status::Success;
        } else {
            return Node::Status::Failure;
        }
    }
};

class IsBlueDetected : public BrainTree::Node {
public:
    Status update() override {
        rgb_raw_t cur_rgb;
        filteredColorSensor->getRawColor(cur_rgb);
        if (cur_rgb.b - cur_rgb.r > 60 && cur_rgb.b <= 255 && cur_rgb.r <= 255) {
            _log("line color changed black to blue.");
            return Node::Status::Success;
        } else {
            return Node::Status::Failure;
        }
    }
};

class IsBlackDetected : public BrainTree::Node {
public:
    Status update() override {
        rgb_raw_t cur_rgb;
        filteredColorSensor->getRawColor(cur_rgb);
        if (cur_rgb.b - cur_rgb.r < 40) {
            _log("line color changed blue to black.");
            return Node::Status::Success;
        } else {
            return Node::Status::Failure;
        }
    }
};

class IsSonarOn : public BrainTree::Node {
public:
    Status update() override {
        int32_t distance = sonarSensor->getDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0)) {
            _log("SONAR_ALERT_DISTANCE=%d", distance);
            return Node::Status::Success;
        } else {
            return Node::Status::Failure;
        }
    }
};

class IsDistanceReached : public BrainTree::Node {
public:
    IsDistanceReached() : flag(false) {}
    Status update() override {
        if (plotter->getDistance() >= BLUE_DISTANCE) {
            if (!flag) {
                _log("BLUE_DISTANCE is reached.");
                flag = true;
            }
            return Node::Status::Success;
        } else {
            return Node::Status::Failure;
        }
    }
private:
    bool flag;
};

class TraceLine : public BrainTree::Node {
public:
    TraceLine() : traceCnt(0) {
        ltPid = new PIDcalculator(P_CONST, I_CONST, D_CONST, PERIOD_UPD_TSK, TURN_MIN, TURN_MAX);
    }
    ~TraceLine() {
        delete ltPid;
    }
    Status update() override {
        int16_t sensor;
        int8_t forward, turn, pwm_L, pwm_R;
        rgb_raw_t cur_rgb;

        filteredColorSensor->getRawColor(cur_rgb);
        sensor = cur_rgb.r;
        /* compute necessary amount of steering by PID control */
        turn = (-1) * _COURSE * ltPid->compute(sensor, (int16_t)GS_TARGET);
        forward = SPEED_NORM;
        /* steer EV3 by setting different speed to the motors */
        pwm_L = forward - turn;
        pwm_R = forward + turn;
        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);
        /* display trace message in every PERIOD_TRACE_MSG ms */
        if (++traceCnt * PERIOD_UPD_TSK >= PERIOD_TRACE_MSG) {
            traceCnt = 0;
            _log("sensor = %d, pwm_L = %d, pwm_R = %d",
                sensor, pwm_L, pwm_R);
            _log("locX = %d, locY = %d, degree = %d, distance = %d",
                (int)plotter->getLocX(), (int)plotter->getLocY(),
                (int)plotter->getDegree(), (int)plotter->getDistance());
        }
        return Node::Status::Running;
    }
protected:
    PIDcalculator* ltPid;
private:
    int traceCnt;
};

class MoveToLine : public BrainTree::Node {
public:
    Status update() override {
        int16_t sensor;
        rgb_raw_t cur_rgb;

        filteredColorSensor->getRawColor(cur_rgb);
        sensor = cur_rgb.r;

        if (sensor >= GS_TARGET) {
            /* move EV3 closer to the line */
            leftMotor->setPWM(SPEED_SLOW);
            rightMotor->setPWM(SPEED_SLOW);
            return Node::Status::Running;
        } else {
            return Node::Status::Success;
        }
    }
};

class RotateEV3 : public BrainTree::Node {
public:
    RotateEV3(int16_t degree) : deltaDegreeTarget(degree),updated(false) {
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
            leftMotor->setPWM(clockwise * SPEED_SLOW);
            rightMotor->setPWM((-clockwise) * SPEED_SLOW);
            return Node::Status::Running;
        } else {
            return Node::Status::Success;
        }
    }
private:
    int16_t deltaDegreeTarget, originalDegree;
    int clockwise;
    bool updated;
};

//sano family add test
class SpinEV3 : public BrainTree::Node { 
public:
    SpinEV3(int direction, int count) : dir(direction), cnt(count) {}
    Status update() override {
        curAngle = gyroSensor->getAngle();
            if(cnt >= 1){
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
                armMotor->setPWM(-50);
                cnt++;
                if(cnt >= 500){
                    return Node::Status::Success;
                }
            }else{
                armMotor->setPWM(30);
                leftMotor->setPWM(23);
                rightMotor->setPWM(25);

                if(curAngle < -9){
                    prevAngle = curAngle;
                }
                if (prevAngle < -9 && curAngle >= 0){
                    ++cnt;
                }
                return Node::Status::Running;
            }
    }
private:
    int8_t dir;
    int cnt;
    int32_t curAngle;
    int32_t prevAngle;
};

class TraceLine2 : public BrainTree::Node {
public:
    TraceLine2(int sp, double kp) : speedVal(sp), kpVal(kp), traceCnt(0), cnt(0) {
        ltPid = new PIDcalculator(kpVal, I_CONST, D_CONST, PERIOD_UPD_TSK, TURN_MIN, TURN_MAX);
    }
    ~TraceLine2() {
        delete ltPid;
    }
    Status update() override {
        int16_t sensor;
        int8_t forward, turn, pwm_L, pwm_R;
        rgb_raw_t cur_rgb;

        filteredColorSensor->getRawColor(cur_rgb);
        sensor = cur_rgb.r;
        /* compute necessary amount of steering by PID control */
        turn = (-1) * _COURSE * ltPid->compute(sensor, (int16_t)GS_TARGET2);
        forward = speedVal;
        /* steer EV3 by setting different speed to the motors */
        pwm_L = forward - turn;
        pwm_R = forward + turn;
        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);
        /* display trace message in every PERIOD_TRACE_MSG ms */
        if (++traceCnt * PERIOD_UPD_TSK >= PERIOD_TRACE_MSG) {
            traceCnt = 0;
            _log("sensor = %d, pwm_L = %d, pwm_R = %d",
                sensor, pwm_L, pwm_R);
            _log("locX = %d, locY = %d, degree = %d, distance = %d",
                (int)plotter->getLocX(), (int)plotter->getLocY(),
                (int)plotter->getDegree(), (int)plotter->getDistance());
        }
        cnt++;
        if(cnt > 2470){
            return Node::Status::Success;
        }else{
            return Node::Status::Running;
        }
    }
protected:
    PIDcalculator* ltPid;
private:
    int traceCnt, speedVal, cnt;
    double kpVal;
};

//sano family add test
class BackRun : public BrainTree::Node { 
public:
    BackRun(int direction, int count) : dir(direction), cnt(count) {}
    Status update() override {

            if(cnt >= 0 && 400 > cnt){
                leftMotor->setBrake(true);
                rightMotor->setBrake(true);
                leftMotor->setPWM(0);
                rightMotor->setPWM(0);
            }
            if(cnt >= 200 && 600 > cnt){
//                leftMotor->setBrake(false);
                leftMotor->setPWM(-9);
                rightMotor->setPWM(-9);
            }else if(cnt >= 600 && cnt < 1200){
                leftMotor->setPWM(9);
                rightMotor->setPWM(1);
            }else if(cnt >= 1200 &&  cnt < 1720){
                leftMotor->setPWM(10);
                rightMotor->setPWM(10);
            }else if(cnt >= 1700 &&  cnt < 2000){
                leftMotor->setPWM(12);
                rightMotor->setPWM(9);
            }else if(cnt >= 2000 && cnt < 4000){
                armMotor->setPWM(30);
            }
            cnt++;
            return Node::Status::Running;
    }
private:
    int8_t dir;
    int cnt;
    int32_t curAngle;
    int32_t prevAngle;
};


/* method to wake up the main task for termination */
class WakeUpMain : public BrainTree::Node {
public:
    Status update() override {
        _log("waking up main...");
        /* wake up the main task */
        // ER ercd = wup_tsk(MAIN_TASK);
        // assert(ercd == E_OK);
        //if (ercd != E_OK) {
        //    syslog(LOG_NOTICE, "wup_tsk() returned %d", ercd);
        //}
        leftMotor->setPWM(0);
        rightMotor->setPWM(0);
        runningMode = 1;
       return Node::Status::Success;
    }
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
    filteredColorSensor = new FilteredColorSensor(PORT_3);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_D);
    armMotor    = new Motor(PORT_A);
    plotter     = new Plotter(leftMotor, rightMotor, gyroSensor);
    /* indicate initialization completion by LED color */
    _log("initialization completed.");
    ev3_led_set_color(LED_ORANGE);

    /* BEHAVIOR TREE DEFINITION */

    /* robot starts line tracing
       when touch sensor is turned on.
       it continues running unless:
         ultrasonic sonar detects an obstacle or
         back button is pressed or
         the second blue part of line is reached at
         further than BLUE_DISTANCE,
       while its location keeps being tracked. */
    tree = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::MemSequence>()
            //.leaf<IsTouchOn>()
            // .leaf<RotateEV3>(30 * _COURSE)
            // .leaf<MoveToLine>()
            // .leaf<RotateEV3>(-30 * _COURSE)
            .composite<BrainTree::ParallelSequence>(1,1)
                .leaf<IsSonarOn>()
                .leaf<IsBackOn>()
                .composite<BrainTree::ParallelSequence>(2,2)
                    .leaf<IsDistanceReached>()
                    .composite<BrainTree::MemSequence>()
                        .leaf<IsBlueDetected>()
                        .leaf<IsBlackDetected>()
                        .leaf<IsBlueDetected>()
                    .end()
                .end()
                .leaf<TraceLine>()
            .end()
            .leaf<WakeUpMain>()
        .end()
        .build();

    //trial - sano family add
    tree_test = (BrainTree::BehaviorTree*) BrainTree::Builder() 
        .composite<BrainTree::MemSequence>()
            .leaf<SpinEV3>(_COURSE, 0) /* TODO magic number */
            .leaf<TraceLine2>(SPEED_SLOW2, P_CONST2)
            .leaf<BackRun>(_COURSE, 0) /* TODO magic number */
            .leaf<WakeUpMain>()
        .end()
        .build();

    /* register cyclic handler to EV3RT */
    sta_cyc(CYC_UPD_TSK);
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
    delete tree;
    /* destroy EV3 objects */
    delete plotter;
    delete armMotor;
    delete tailMotor;
    delete rightMotor;
    delete leftMotor;
    delete gyroSensor;
    delete filteredColorSensor;
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
    filteredColorSensor->sense();
    plotter->plot();

    if(runningMode==1){
        if (tree_test != nullptr) tree_test->update();
    }else{
        if (tree != nullptr) tree->update();
    }
}