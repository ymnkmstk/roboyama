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
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Motor*          armMotor;

BrainTree::BehaviorTree* tree = nullptr;

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
        colorSensor->getRawColor(cur_rgb);
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
        colorSensor->getRawColor(cur_rgb);
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
            _log("SONAR_ALERT_DISTANCE");
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
        /* read variables from Blackboard */
        double distance = blackboard->getDouble(STR(BoardItem.DIST));
        if (distance >= BLUE_DISTANCE) {
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

class EstimateLocation : public BrainTree::Node {
public:
    EstimateLocation() : distance(0.0),azimuth(0.0),locX(0.0),locY(0.0),traceCnt(0) {
        /* reset motor encoders */
        leftMotor->reset();
        rightMotor->reset();
        /* reset gyro sensor */
        gyroSensor->reset();
        /* initialize variables */
        prevAngL = leftMotor->getCount();
        prevAngR = rightMotor->getCount();
    }
    Status update() override {
        /* accumulate distance */
        int32_t curAngL = leftMotor->getCount();
        int32_t curAngR = rightMotor->getCount();
        double deltaDistL = M_PI * TIRE_DIAMETER * (curAngL - prevAngL) / 360.0;
        double deltaDistR = M_PI * TIRE_DIAMETER * (curAngR - prevAngR) / 360.0;
        double deltaDist = (deltaDistL + deltaDistR) / 2.0;
        distance += deltaDist;
        prevAngL = curAngL;
        prevAngR = curAngR;
        /* calculate azimuth */
        double deltaAzi = atan2((deltaDistL - deltaDistR), WHEEL_TREAD);
        azimuth += deltaAzi;
        if (azimuth > M_TWOPI) {
            azimuth -= M_TWOPI;
        } else if (azimuth < 0.0) {
            azimuth += M_TWOPI;
        }
        /* estimate location */
        locX += (deltaDist * sin(azimuth));
        locY += (deltaDist * cos(azimuth));
        /* write variables to Blackboard for the use by other actions */
        blackboard->setDouble(STR(BoardItem.LOCX), locX);
        blackboard->setDouble(STR(BoardItem.LOCY), locY);
        blackboard->setDouble(STR(BoardItem.DIST), distance);
        /* display trace message in every PERIOD_TRACE_MSG ms */
        if (++traceCnt * PERIOD_UPD_TSK >= PERIOD_TRACE_MSG) {
            traceCnt = 0;
            _log("locX = %d, locY = %d, distance = %d",
                (int)locX, (int)locY, (int)distance);
        }
        return Node::Status::Running;
    }
protected:
    double distance, azimuth, locX, locY;
    int32_t prevAngL, prevAngR;
private:
    int traceCnt;
};

class TraceLine : public BrainTree::Node {
public:
    TraceLine() : fillFIR(FIR_ORDER + 1), traceCnt(0) {
        ltPid = new PIDcalculator(P_CONST, I_CONST, D_CONST, PERIOD_UPD_TSK, TURN_MIN, TURN_MAX);
        fir_r = new FIR_Transposed<FIR_ORDER>(hn);
        fir_g = new FIR_Transposed<FIR_ORDER>(hn);
        fir_b = new FIR_Transposed<FIR_ORDER>(hn);
    }
    ~TraceLine() {
        delete fir_b;
        delete fir_g;
        delete fir_r;
        delete ltPid;
    }
    Status update() override {
        int16_t sensor;
        int8_t forward, turn, pwm_L, pwm_R;
        rgb_raw_t cur_rgb;

        colorSensor->getRawColor(cur_rgb);
        /* process RGB by the Low Pass Filter */
        cur_rgb.r = fir_r->Execute(cur_rgb.r);
        cur_rgb.g = fir_g->Execute(cur_rgb.g);
        cur_rgb.b = fir_b->Execute(cur_rgb.b);

        /* wait until FIR array is filled */
        if (fillFIR > 0) {
            fillFIR--;
        } else {
            sensor = cur_rgb.r;

            /* compute necessary amount of steering by PID control */
            turn = _EDGE * ltPid->compute(sensor, (int16_t)GS_TARGET);
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
            }
        }
        return Node::Status::Running;
    }
protected:
    PIDcalculator* ltPid;
    FIR_Transposed<FIR_ORDER> *fir_r, *fir_g, *fir_b;
private:
    int traceCnt, fillFIR;
};

class MoveToLine : public BrainTree::Node {
public:
    MoveToLine() : fillFIR(FIR_ORDER + 1) {
        fir_r = new FIR_Transposed<FIR_ORDER>(hn);
        fir_g = new FIR_Transposed<FIR_ORDER>(hn);
        fir_b = new FIR_Transposed<FIR_ORDER>(hn);
    }
    ~MoveToLine() {
        delete fir_b;
        delete fir_g;
        delete fir_r;
    }
    Status update() override {
        int16_t sensor;
        rgb_raw_t cur_rgb;

        colorSensor->getRawColor(cur_rgb);
        /* process RGB by the Low Pass Filter */
        cur_rgb.r = fir_r->Execute(cur_rgb.r);
        cur_rgb.g = fir_g->Execute(cur_rgb.g);
        cur_rgb.b = fir_b->Execute(cur_rgb.b);

        /* wait until FIR array is filled */
        if (fillFIR > 0) {
            fillFIR--;
        } else {
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
    }
protected:
    FIR_Transposed<FIR_ORDER> *fir_r, *fir_g, *fir_b;
private:
    int fillFIR;
};

class RotateEV3 : public BrainTree::Node {
public:
    RotateEV3(int direction, int count) : dir(direction), cnt(count) {}
    Status update() override {
        if (--cnt >= 0) {
            leftMotor->setPWM((-dir) * SPEED_SLOW);
            rightMotor->setPWM(dir * SPEED_SLOW);
            return Node::Status::Running;
        } else {
            return Node::Status::Success;
        }
    }
private:
    int8_t dir;
    int cnt;
};

/* method to wake up the main task for termination */
class WakeUpMain : public BrainTree::Node {
public:
    Status update() override {
        _log("waking up main...");
        /* wake up the main task */
        ER ercd = wup_tsk(MAIN_TASK);
        assert(ercd == E_OK);
        if (ercd != E_OK) {
            syslog(LOG_NOTICE, "wup_tsk() returned %d", ercd);
        }
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
    colorSensor = new ColorSensor(PORT_3);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_D);
    armMotor    = new Motor(PORT_A);
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
            .leaf<IsTouchOn>()
            .leaf<RotateEV3>(_EDGE, 20) /* TODO magic number */
            .leaf<MoveToLine>()
            .leaf<RotateEV3>((-1) * _EDGE, 20) /* TODO magic number */
            .composite<BrainTree::ParallelSequence>(1,1)
                .leaf<EstimateLocation>()
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
    if (tree != nullptr) tree->update();
}