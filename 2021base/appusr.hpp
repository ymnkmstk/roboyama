/*
    appusr.hpp

    Copyright © 2021 Wataru Taniguchi. All rights reserved.
*/
#ifndef appusr_hpp
#define appusr_hpp

#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Steering.h"
#include "Clock.h"
using namespace ev3api;

#include "etroboc_ext.h"

/* M_PI and M_TWOPI is NOT available even with math header file under -std=c++11
   because they are not strictly comforming to C++11 standards
   this program is compiled under -std=gnu++11 option */
#include <math.h>

#include "BrainTree.h"
#include "FilteredColorSensor.hpp"
#include "Plotter.hpp"
#include "PIDcalculator.hpp"

/* global variables */
extern FILE*        bt;
extern Clock*       clock;
extern TouchSensor* touchSensor;
extern SonarSensor* sonarSensor;
extern GyroSensor*  gyroSensor;
extern Motor*       leftMotor;
extern Motor*       rightMotor;
extern Motor*       tailMotor;
extern Motor*       armMotor;
extern FilteredColorSensor* filteredColorSensor;

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

//#define LOG_ON_CONSOL

/* ##__VA_ARGS__ is gcc proprietary extention.
   this is also where -std=gnu++11 option is necessary */
#ifdef LOG_ON_CONSOL
#define _log(fmt, ...) \
    syslog(LOG_NOTICE, "%08u, %s: " fmt, \
    clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__)
#else
#define _log(fmt, ...) \
    fprintf(bt, "%08u, %s: " fmt "\n", \
    clock->now(), __PRETTY_FUNCTION__, ##__VA_ARGS__)
#endif

/* macro to covert an enumeration constant to a string */
#define STR(var) #var

/* macro for making program compatible for both left and right courses.
   the default is left course. */ 
#if defined(MAKE_RIGHT)
    static const int _COURSE = -1;
#else
    static const int _COURSE = 1;
#endif

#define PERIOD_TRACE_MSG   1000 * 1000 /* Trace message in every 1000 ms    */
#define P_CONST           0.85D
#define I_CONST     0.00000001D
#define D_CONST            0.5D
#define TURN_MIN            -16  /* minimum value PID calculator returns    */
#define TURN_MAX             16  /* maximum value PID calculator returns    */
#define SPEED_SLOW           15
#define SPEED_NORM           25  /* was 50 for 2020 program                 */
#define GS_TARGET            47  /* was 47 for 2020 program                 */
#define SONAR_ALERT_DISTANCE 10  /* in centimeters                          */
#define BLUE_DISTANCE     10000  /* 2nd blue part should be further than this   */ 

enum BoardItem {
    LOCX, /* horizontal location    */
    LOCY, /* virtical   location    */
    DIST, /* accumulated distance   */
};

#endif /* appusr_hpp */