/*
    TrapezoidalMtrControler.hpp
    Need to creat two objects for left wheel and right wheel
    
    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef TrapezoidalMtrControler_hpp
#define TrapezoidalMtrControler_hpp

#define CNT_INTERVAL    3       /* hoge */


class TrapezoidalMtrControler{
public:
    TrapezoidalMtrControler();
    ~TrapezoidalMtrControler();
   // void setPwm(int argPwm);
    int32_t getPwm(int startpwm ,int endpwm ,int lr);

protected:
    //ev3api::Motor *leftMotor, *rightMotor;
    int32_t trapezoidalDriveCalc(int startpwm ,int endpwm ,int lr);
    int32_t cnt,startPwm,endPwm,pwm;
};

#endif /* TrapezoidalMtrControler_hpp */