/*
    app.cpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#include "app.h"
#include "appusr.hpp"

/* global variables */
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
            syslog(LOG_NOTICE, "%08u, IsTouchOn::update() : TouchSensor pressed", clock->now());
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
            syslog(LOG_NOTICE, "%08u, IsSonarOn::update() : SONAR_ALERT_DISTANCE", clock->now());
            return Node::Status::Success;
        } else {
            return Node::Status::Failure;
        }
    }
};

class EstimateLocation : public BrainTree::Node {
public:
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

        /* display trace message in every PERIOD_TRACE_MSG ms */
        if (++traceCnt * PERIOD_UPD_TSK >= PERIOD_TRACE_MSG) {
            traceCnt = 0;
            _debug(syslog(LOG_NOTICE, "%08u, EstimateLocation::update() : locX = %d, locY = %d, distance = %d",
                clock->now(), (int)locX, (int)locY, (int)distance));
        }

        return Node::Status::Success;
    }
protected:
    double distance, azimuth, locX, locY;
    int32_t prevAngL, prevAngR;
private:
    int traceCnt = 0;
};

class WakeUpMain : public BrainTree::Node {
public:
    Status update() override {
        syslog(LOG_NOTICE, "%08u, WakeUpMain::update() : Ending...", clock->now());
        /* wake up the main task */
        ER ercd = wup_tsk(MAIN_TASK);
        assert(ercd == E_OK);
        return Node::Status::Success;
    }
};

class TraceLine : public BrainTree::Node
{
public:
    void initialize() {
        ltPid = new PIDcalculator(P_CONST, I_CONST, D_CONST, PERIOD_UPD_TSK, TURN_MIN, TURN_MAX);
        fir_r = new FIR_Transposed<FIR_ORDER>(hn);
        fir_g = new FIR_Transposed<FIR_ORDER>(hn);
        fir_b = new FIR_Transposed<FIR_ORDER>(hn);
        fillFIR = FIR_ORDER + 1;
    }
    void terminate(Status s) {
        delete fir_b;
        delete fir_g;
        delete fir_r;
        delete ltPid;
    }
    Status update() override {
        int16_t sensor, background, grayScale, grayScaleBlueless;
        int8_t forward, turn, pwm_L, pwm_R;
        rgb_raw_t cur_rgb;
        hsv_raw_t cur_hsv;

        colorSensor->getRawColor(cur_rgb);
        /* process RGB by the Low Pass Filter */
        cur_rgb.r = fir_r->Execute(cur_rgb.r);
        cur_rgb.g = fir_g->Execute(cur_rgb.g);
        cur_rgb.b = fir_b->Execute(cur_rgb.b);

        /* wait until FIR array is filled */
        if (fillFIR > 0) {
            fillFIR--;
        } else {
            rgb_to_hsv(cur_rgb, cur_hsv);
            grayScale = (cur_rgb.r * 77 + cur_rgb.g * 150 + cur_rgb.b * 29) / 256;
            /* B - G cuts off blue */
            grayScaleBlueless = (cur_rgb.r * 77 + cur_rgb.g * 150 + (cur_rgb.b - cur_rgb.g) * 29) / 256;

            background = colorSensor->getAmbient();

            /* compute necessary amount of steering by PID control */
            turn = _EDGE * ltPid->compute(grayScaleBlueless, (int16_t)GS_TARGET);
            forward = SPEED_NORM;

            /* steer EV3 by setting different speed to the motors */
            pwm_L = forward - turn;
            pwm_R = forward + turn;
            leftMotor->setPWM(pwm_L);
            rightMotor->setPWM(pwm_R);

            /* display trace message in every PERIOD_TRACE_MSG ms */
            if (++traceCnt * PERIOD_UPD_TSK >= PERIOD_TRACE_MSG) {
                traceCnt = 0;
                _debug(syslog(LOG_NOTICE, "%08u, TraceLine::update() : sensor = %d, background = %d",
                    clock->now(), grayScaleBlueless, background));
                _debug(syslog(LOG_NOTICE, "%08u, TraceLine::update() : pwm_L = %d, pwm_R = %d",
                    clock->now(), pwm_L, pwm_R));
            }
        }
        return Node::Status::Running;
    }
protected:
    PIDcalculator*  ltPid;
    FIR_Transposed<FIR_ORDER> *fir_r, *fir_g, *fir_b;

    void rgb_to_hsv(rgb_raw_t rgb, hsv_raw_t& hsv) {
        uint16_t max, min;
        double cr, cg, cb, h;  /* must be double */

        max = rgb.r;
        if(max < rgb.g) max = rgb.g;
        if(max < rgb.b) max = rgb.b;
    
        min = rgb.r;
        if(min > rgb.g) min = rgb.g;
        if(min > rgb.b) min = rgb.b;
    
        hsv.v = 100 * max / (double)255.0;
    
        if (!max) {
            hsv.s = 0;
            hsv.h = 0;
        } else {
            hsv.s = 100 * (max - min) / (double)max;
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
private:
    int traceCnt = 0, fillFIR;
};

/* a cyclic handler to activate a task */
void task_activator(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK || E_QOVR);

    if (ercd != E_OK) {
        _debug(syslog(LOG_NOTICE, "%08lu, act_tsk() returned %d", clock->now(), ercd));
    }
}

void main_task(intptr_t unused) {
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
    
    /* display message on LCD */
    /*
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET", 0, CALIB_FONT_HEIGHT*1);
    */

    /* indicate initialization completion by LED color */
    _debug(syslog(LOG_NOTICE, "%08u, Initialization completed.", clock->now()));
    /*
    ev3_led_set_color(LED_ORANGE);
    */

    /* BEHAVIOR TREE DEFINITION */
    tree = (BrainTree::BehaviorTree*) BrainTree::Builder()
        .composite<BrainTree::MemSequence>()
            .leaf<IsTouchOn>()
            .decorator<BrainTree::UntilFailure>()
                .composite<BrainTree::Sequence>()
                    .leaf<EstimateLocation>()
                    .decorator<BrainTree::Inverter>()
                        .leaf<IsSonarOn>()
                    .end()
                    .leaf<TraceLine>()
                .end()
            .end()
            .leaf<WakeUpMain>()
        .end()
        .build();

    /* register cyclic handler to EV3RT */
    sta_cyc(CYC_UPD_TSK);

    /* sleep until being waken up */
    ER ercd = slp_tsk();
    assert(ercd == E_OK);

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
    ext_tsk();
}

/* periodic task to update the behavior tree */
void update_task(intptr_t unused) {
    if (tree != NULL) tree->update();
}