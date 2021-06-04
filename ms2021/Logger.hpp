/*
    Logger.hpp
    
    Copyright © 2021 Tomoko Sano. All rights reserved.
*/
#ifndef Logger_hpp
#define Logger_hpp

class Logger{
public:
    Logger(Plotter* pltr,FilteredMotor* lm, FilteredMotor* rm, ev3api::GyroSensor* gs,FilteredColorSensor* cs,ev3api::SonarSensor* ss,ev3api::Clock* ck);
    ~Logger();
    void outputLog(bool runningBool ,bool slalomBool, bool garageBool,int logInterval,int state);

protected:
    FilteredMotor* leftMotor;
    FilteredMotor* rightMotor;
    ev3api::GyroSensor* gyroSensor;
    FilteredColorSensor* colorSenser;
    Plotter* plotter;
    ev3api::SonarSensor* sonarSenser;
    ev3api::Clock* clock;
    int traceCnt;
    int32_t prevAngL, prevAngR;
};

#endif /* Logger_hpp */