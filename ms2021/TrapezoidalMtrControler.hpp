/*
    TrapezoidalMtrControler.hpp
    Need to creat two objects for left wheel and right wheel
    
    Copyright © 2021 Tomoko Sano. All rights reserved.
*/
#ifndef TrapezoidalMtrControler_hpp
#define TrapezoidalMtrControler_hpp

class TrapezoidalMtrControler{
public:
    TrapezoidalMtrControler();
    ~TrapezoidalMtrControler();
   // void setPwm(int argPwm);
    int32_t getPwm(int startpwm ,int endpwm, int cntinterval, int cntpwd);

protected:
    //ev3api::Motor *leftMotor, *rightMotor;
    int32_t trapezoidalDriveCalc(int startpwm ,int endpwm,  int cntinterval, int cntpwd);
    int32_t cnt,startPwm,endPwm,pwm,cntInterval,cntPwd;
};

#endif /* TrapezoidalMtrControler_hpp */