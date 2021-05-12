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

int32_t TrapezoidalMtrControler::getPwm(int startpwm ,int endpwm,int lr){
     return trapezoidalDriveCalc(startpwm,endpwm,lr);
}


//FilterdMoterができたら、そこにpwmのgetメソッドと追加。同時にstartpwm引数を削除
//lr = 1:left 2:right
int32_t TrapezoidalMtrControler::trapezoidalDriveCalc(int startpwm ,int endpwm,int lr){

    if(endPwm != endpwm){
        cnt = 0;
        endPwm = endpwm;
        
        pwm = startpwm; //仮
        // if(lr = 1){
        //     pwm = leftMotor->getPWM();
        // }else{
        //     pwm = rightMotor->getPWM();
        // }

    }else{

        //in case of acceletion
        if(pwm < endPwm){
            if(cnt % CNT_INTERVAL == 0){
                pwm++;
            }

        //in case of decelerate
        }else if(pwm > endPwm){
            if(cnt % CNT_INTERVAL == 0){
                pwm--;
            }

        }else if(pwm == endPwm){
            //printf("pwmがendPwmに到達しました:%d\n",pwm);
        }
        cnt++;
    }
    return pwm;
}

