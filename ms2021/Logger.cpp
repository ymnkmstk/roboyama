/*
    Logger.cpp

    Copyright © 2021 Tomoko Sano. All rights reserved.
*/

#include "appusr.hpp"
#include "Logger.hpp"

Logger::Logger(Plotter* pltr,FilteredMotor* lm, FilteredMotor* rm, ev3api::GyroSensor* gs, FilteredColorSensor* cs, ev3api::SonarSensor* ss, ev3api::Clock* ck) :
plotter(pltr),leftMotor(lm),rightMotor(rm),gyroSensor(gs),colorSenser(cs) ,sonarSenser(ss),clock(ck){
    traceCnt = 0;
}

Logger::~Logger() {}

void Logger::outputLog(bool runningBool ,bool slalomBool, bool garageBool,int logInterval,int state){

    rgb_raw_t cur_rgb;
    colorSensor->getRawColor(cur_rgb);


    if (++traceCnt  >= logInterval) {
        traceCnt = 0;
        if((runningBool && state == ST_running) || 
                (slalomBool && state == ST_slalom) || 
                        (garageBool && state == ST_garage)){
                
                _log("pwdL = %d; pwdR = %d; time(sec) = %d; distance = %d; degree = %d; locX = %d; locY = %d; sonarDistance = %d; gyroAngle = %d; gyroAnglerVelocity = %d; red = %d; green = %d; blue = %d"
                    ,leftMotor->getPwm()
                    ,rightMotor->getPwm()
                    //,round((int32_t)clock->now()/10000)
                    ,clock->now()
                    ,(int)plotter->getDistance()
                    ,(int)plotter->getDegree()
                    ,(int)plotter->getLocX()
                    ,(int)plotter->getLocY()
                    ,sonarSenser->getDistance()
                    ,gyroSensor->getAngle()
                    ,gyroSensor->getAnglerVelocity()
                    ,cur_rgb.r
                    ,cur_rgb.g
                    ,cur_rgb.b
                );
    }
    }
}

