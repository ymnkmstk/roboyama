/*
    Logger.cpp

    Copyright © 2021 Tomoko Sano / MS Mode 2. All rights reserved.
*/

#include "Logger.hpp"

Logger::Logger() : traceCnt(0) {}

void Logger::outputLog(int logInterval){
    if (logInterval == 0) return; /* do nothing */
 
    if (++traceCnt  >= logInterval) {
        traceCnt = 0;

        rgb_raw_t cur_rgb;
        colorSensor->getRawColor(cur_rgb);
        int32_t angL = plotter->getAngL();
        int32_t angR = plotter->getAngR();

        _log("pwdL = %d; pwdR = %d; time(sec) = %d; distance = %d; degree = %d; locX = %d; locY = %d; sonarDistance = %d; gyroAngle = %d; gyroAnglerVelocity = %d; red = %d; green = %d; blue = %d; deltaAngDiff = %d"
            ,leftMotor->getPWM()
            ,rightMotor->getPWM()
            ,clock->now()
            ,(int)plotter->getDistance()
            ,(int)plotter->getDegree()
            ,(int)plotter->getLocX()
            ,(int)plotter->getLocY()
            ,sonarSensor->getDistance()
            ,gyroSensor->getAngle()
            ,gyroSensor->getAnglerVelocity()
            ,cur_rgb.r
            ,cur_rgb.g
            ,cur_rgb.b
            ,(int)((angL-prevAngL)-(angR-prevAngR))
        );

        prevAngL = angL;
        prevAngR = angR;
    }
}