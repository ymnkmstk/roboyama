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
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Motor*          armMotor;
Plotter*        plotter;

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

class IsBlueDetected : public BrainTree::Node {
public:
    Status update() override {
        rgb_raw_t cur_rgb;
        colorSensor->getRawColor(cur_rgb);
        if (cur_rgb.b - cur_rgb.r > 60 && cur_rgb.b <= 255 && cur_rgb.r <= 255) {
            _log("line color changed black to blue.");
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
};

class IsBlackDetected : public BrainTree::Node {
public:
    Status update() override {
        rgb_raw_t cur_rgb;
        colorSensor->getRawColor(cur_rgb);
        if (cur_rgb.b - cur_rgb.r < 40) {
            _log("line color changed blue to black.");
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
};

class IsSonarOn : public BrainTree::Node {
public:
    Status update() override {
        int32_t distance = sonarSensor->getDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0)) {
            _log("SONAR_ALERT_DISTANCE");
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
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
    int32_t target_distance;
    bool updated, earned;
};

class TraceLine : public BrainTree::Node {
public:
    TraceLine(int s, int t, double p, double i, double d) : speed(s),target(t),traceCnt(0),prevAngL(0),prevAngR(0) {
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
        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);
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
    }
protected:
    int speed, target;
    PIDcalculator* ltPid;
    int32_t prevAngL, prevAngR;
private:
    int traceCnt;
};

/*  usage:
    ".leaf<MoveToLine>(SPEED_SLOW)"
    is to move robot straight ahead until a line is detected by speed SPEED_SLOW  */
class MoveToLine : public BrainTree::Node {
public:
    MoveToLine(int s) : speed(s) {}
    Status update() override {
        int16_t sensor;
        rgb_raw_t cur_rgb;

        colorSensor->getRawColor(cur_rgb);
        sensor = cur_rgb.r;

        if (sensor >= GS_TARGET) {
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
    int speed;
    int32_t prevAngL, prevAngR;
private:
    int traceCnt;
};

/*  usage:
    ".leaf<RotateEV3>(30 * _COURSE)"
    is to rotate robot 30 degrees clockwise when in L course */
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
            return Status::Running;
        } else {
            return Status::Success;
        }
    }
private:
    int16_t deltaDegreeTarget, originalDegree;
    int clockwise;
    bool updated;
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
                if(cnt >= 200){
                    return Status::Success;
                }
                return Status::Running;
            }else{
                armMotor->setPWM(30);
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
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_D);
    armMotor    = new Motor(PORT_A);
    plotter     = new Plotter(leftMotor, rightMotor, gyroSensor);

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
            .leaf<IsSonarOn>()
            .leaf<IsBackOn>()
            .composite<BrainTree::ParallelSequence>(2,2)
                .leaf<IsDistanceEarned>(BLUE_DISTANCE)
                .composite<BrainTree::MemSequence>()
                    .leaf<IsBlueDetected>()
                    .leaf<IsBlackDetected>()
                    .leaf<IsBlueDetected>()
                .end()
            .end()
            .leaf<TraceLine>(SPEED_NORM, GS_TARGET, P_CONST, I_CONST, D_CONST)
        .end()
        .build();

    tr_slalom = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::MemSequence>()
            .leaf<ClimbBoard>(_COURSE, 0)
            .composite<BrainTree::ParallelSequence>(1,2)
                .leaf<IsDistanceEarned>(1200)
                .leaf<TraceLine>(SPEED_SLOW, GS_TARGET2, P_CONST2, I_CONST2, D_CONST2)
            .end()
        .end()
        .build();

    tr_garage = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::MemSequence>()
            .leaf<RotateEV3>(-30 * _COURSE)
            .leaf<RotateEV3>(30 * _COURSE)
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
                state = ST_running;
                _log("State changed: ST_calibration to ST_running");
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
}