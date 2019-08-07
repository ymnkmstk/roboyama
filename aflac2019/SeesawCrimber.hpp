//
//  SeesawCrimber.hpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef SeesawCrimber_hpp
#define SeesawCrimber_hpp

#include "crew.hpp"

#define TAIL_ANGLE_NORMAL_RUN    80  /* 通常走行時のしっぽの角度[度] */
#define TAIL_ANGLE_SEESAW_FIRST  40  /* シーソー上り始めのしっぽの角度[度] */
#define START_SPEED 0  /* START時の各モーターに与える値 */

class SeesawCrimber : public LineTracer {
private:
	int16_t startTime;
	int16_t counter;
	int16_t speed;

protected:
public:
    SeesawCrimber();
    SeesawCrimber(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~SeesawCrimber();
};

#endif /* SeesawCrimber_hpp */
