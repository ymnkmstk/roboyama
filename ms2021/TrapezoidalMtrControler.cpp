/*
    TrapezoidalMtrControler.cpp

    Copyright © 2021 Tomoko Sano. All rights reserved.
*/

#include "appusr.hpp"
#include "TrapezoidalMtrControler.hpp"

TrapezoidalMtrControler::TrapezoidalMtrControler(){
    cnt = 0;
    pwm = 0;
    endPwm = 0;
}

TrapezoidalMtrControler::~TrapezoidalMtrControler() {}

int32_t TrapezoidalMtrControler::getPwm(int startpwm ,int endpwm, int cntinterval, int cntpwd){
    //_log("getPwm: cntinterval = %d, cntpwd = %d\n",cntinterval,cntpwd);
     return trapezoidalDriveCalc(startpwm,endpwm,cntinterval,cntpwd);
}



int32_t TrapezoidalMtrControler::trapezoidalDriveCalc(int startpwm ,int endpwm, int cntinterval, int cntpwd){

    //_log("trapezoidalDriveCalc: cntinterval = %d, cntpwd = %d\n",cntinterval,cntpwd);

    if(endPwm != endpwm){
        cnt = 0;
        endPwm = endpwm;        
        pwm = startpwm; 

    }else{

        if(cntinterval == 0){
            cntInterval = C_INTERVAL;
        }else{
            cntInterval = cntinterval;
        }
        if(cntpwd == 0){
            cntPwd = C_PWD;
        }else{
            cntPwd = cntpwd;
        }

        //in case of acceletion
        if(pwm < endPwm){
            if(cnt % cntInterval == 0){
                pwm = pwm + cntPwd;
                if(pwm > endPwm){
                    pwm = endPwm;
                }
            }

        //in case of decelerate
        }else if(pwm > endPwm){
            if(cnt % cntInterval == 0){
                pwm = pwm - cntPwd;
                if(pwm < endPwm){
                    pwm = endPwm;
                }                
            }
        }
        cnt++;
    }
    return pwm;
}

