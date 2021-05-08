/*
    Plotter.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef Plotter_hpp
#define Plotter_hpp

#define TIRE_DIAMETER     90.0F  /* diameter of tire in milimater           */
#define WHEEL_TREAD      140.0F  /* distance between right and left wheels  */
#define ACCELERATION_RATE_L    0.93  /* sano family */
#define ACCELERATION_RATE_R    1.07  /* sano family */
#define PWD_RANGE              5     /* sano family */


#include "GyroSensor.h"
#include "Motor.h"

/* M_PI and M_TWOPI is NOT available even with math header file under -std=c++11
   because they are not strictly comforming to C++11 standards
   this program is compiled under -std=gnu++11 option */
#include <math.h>

class Plotter {
public:
    Plotter(ev3api::Motor* lm, ev3api::Motor* rm, ev3api::GyroSensor* gs);
    int32_t getDistance();
    int16_t getAzimuth();
    int16_t getDegree();
    int32_t getLocX();
    int32_t getLocY();
    int32_t getAngL();
    int32_t getAngR();
    //void plot();
    void setPwmL(int pwm);
    void setPwmR(int pwm);
    double getprmDeltaDistL();
    double getprmDeltaDistR();   
    void plot(int startMode);
protected:
    ev3api::Motor *leftMotor, *rightMotor;
    ev3api::GyroSensor *gyroSensor;
    double distance, azimuth, locX, locY,prmDeltaDistL,prmDeltaDistR,sumDeltaDistL,sumDeltaDistR;
    int32_t prevAngL, prevAngR;
    int prevLeftPwm,prevRightPwm,leftPwm,rightPwm,courseStep;
};

#endif /* Plotter_hpp */