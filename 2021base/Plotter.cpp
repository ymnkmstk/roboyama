/*
    Plotter.cpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#include "Plotter.hpp"

Plotter::Plotter(ev3api::Motor* lm, ev3api::Motor* rm, ev3api::GyroSensor* gs) :
distance(0.0),azimuth(0.0),locX(0.0),locY(0.0),leftMotor(lm),rightMotor(rm),gyroSensor(gs) {
    /* reset motor encoders */
    leftMotor->reset();
    rightMotor->reset();
    /* reset gyro sensor */
    gyroSensor->reset();
    /* initialize variables */
    prevAngL = leftMotor->getCount();
    prevAngR = rightMotor->getCount();
    courseStep=0;
    sumDeltaDistL = 0.0;
    sumDeltaDistR = 0.0;
}

int32_t Plotter::getDistance() {
    return (int32_t)distance;
}

int16_t Plotter::getAzimuth() {
    return (int32_t)azimuth;
}

int16_t Plotter::getDegree() {
    // degree = 360.0 * radian / M_TWOPI;
    int16_t degree = (360.0 * azimuth / M_TWOPI);
    return degree;
}

int32_t Plotter::getLocX() {
    return (int32_t)locX;
}

int32_t Plotter::getLocY() {
    return (int32_t)locY;
}

int32_t Plotter::getAngL() {
    return prevAngL;
}

int32_t Plotter::getAngR() {
    return prevAngR;
}

//sano family added
double Plotter::getprmDeltaDistL() {
    return prmDeltaDistL;
}

double Plotter::getprmDeltaDistR() {
    return prmDeltaDistR;
}

void Plotter::setPwmL(int pwm) {
    leftPwm =pwm ;
}
void Plotter::setPwmR(int pwm) {
    rightPwm =pwm ;
}

void Plotter::plot(int startMode) {


    /* accumulate distance */
    int32_t curAngL = leftMotor->getCount();
    int32_t curAngR = rightMotor->getCount();
    double deltaDistL = M_PI * TIRE_DIAMETER * (curAngL - prevAngL) / 360.0;
    double deltaDistR = M_PI * TIRE_DIAMETER * (curAngR - prevAngR) / 360.0;

    double a,b,c,d = 0;
    // if(distance > 1600){
    //     deltaDistR = deltaDistR * sumDeltaDistL / sumDeltaDistR;
    // }

    // //add sano family
    // if(leftPwm - prevLeftPwm >= 10){
    //     deltaDistL = deltaDistL + 1;
    // }else if(leftPwm - prevLeftPwm <= -10){
    //     deltaDistL = deltaDistL - 1;
    // }
    // if(rightPwm - prevRightPwm >= 10){
    //      deltaDistR = deltaDistR + 1;
    // }else if(rightPwm - prevRightPwm <= -10){
    //      deltaDistR = deltaDistR - 1;
    // }
    // //add sano family

    //add sano family


    if(leftPwm - prevLeftPwm > 5){
        if(leftPwm < rightPwm){
            a = (leftPwm - prevLeftPwm) * 0.93;
        }else if(leftPwm > rightPwm){
            a = (leftPwm - prevLeftPwm) * 1.08;
        }else{
            a = (leftPwm - prevLeftPwm) * 1;
        }
    }else if(leftPwm - prevLeftPwm < -5){
        a = (leftPwm - prevLeftPwm) * 1;
    }


    if(rightPwm - prevRightPwm > 5){
        if(leftPwm < rightPwm){
            b = (rightPwm - prevRightPwm) * 1.08;
        }else if(leftPwm > rightPwm){
            b = (rightPwm - prevRightPwm) * 0.93;
        }else{
            b = (rightPwm - prevRightPwm) * 1;
        }
    }else if(rightPwm - prevRightPwm < -5){
        b = (rightPwm - prevRightPwm) * 1;
    }

    if(rightPwm - leftPwm >= 70 ){
        c = (rightPwm - leftPwm) * 0.0056;
        // d = 0;
    }else if(rightPwm - leftPwm>=40 && rightPwm - leftPwm < 75){
        c = (rightPwm - leftPwm) * 0.00539;
    }else if(leftPwm - rightPwm >= 70 ){
        // c = 0;
        d = (leftPwm - rightPwm) * 0.0056;
    }else if(leftPwm-rightPwm>=40 && leftPwm -rightPwm < 75){
        //c = rightPwm * 0.02125;
        d = (leftPwm - rightPwm) * 0.00539;
    }

    prmDeltaDistL = deltaDistL;
    prmDeltaDistR = deltaDistR;
    prevLeftPwm =  leftPwm;
    prevRightPwm = rightPwm;

    deltaDistL = deltaDistL + (a * 0.095) + d;
    deltaDistR = deltaDistR + (b * 0.095) + c;

    //add sano family


    double deltaDist = (deltaDistL + deltaDistR) / 2.0;
    distance += deltaDist;
    prevAngL = curAngL;
    prevAngR = curAngR;
    /* calculate azimuth */
    double deltaAzi = (deltaDistL - deltaDistR) / WHEEL_TREAD;
    azimuth += deltaAzi;
    if (azimuth > M_TWOPI) {
        azimuth -= M_TWOPI;
    } else if (azimuth < 0.0) {
        azimuth += M_TWOPI;
    }
    /* estimate location */
    locX += (deltaDist * sin(azimuth));
    locY += (deltaDist * cos(azimuth));

    //sano family added
    if(startMode == 1 && distance < 1600){
        azimuth = 0.0;
        locX = 0.0;
        locY = distance;
        courseStep = 1;
    }else if(startMode == 1 && distance > 1600 && courseStep == 1){ 
         courseStep = 2;
    }

    // if(distance >= 140 && distance < 1600){
    //     sumDeltaDistL = sumDeltaDistL + deltaDistL;
    //     sumDeltaDistR = sumDeltaDistR + deltaDistR;
    // }

}