//
//  Captain.hpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#ifndef Captain_hpp
#define Captain_hpp

#include "aflac_common.hpp"
#include "BlindRunner.hpp"

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

class Captain {
private:
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;
    ColorSensor*    colorSensor;
    GyroSensor*     gyroSensor;
    Motor*          leftMotor;
    Motor*          rightMotor;
    Motor*          tailMotor;
    Steering*       steering;
    LineTracer*     lineTracer;
    BlindRunner*    blindRunner;
protected:
public:
    Captain();
    void takeoff();
    void decide(uint8_t event);
    void triggerLanding();
    void land();
    ~Captain();
};

extern Captain* captain;

#endif /* Captain_hpp */