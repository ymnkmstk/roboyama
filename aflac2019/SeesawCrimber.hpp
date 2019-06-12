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

class SeesawCrimber : public Navigator {
protected:
public:
    SeesawCrimber();
    SeesawCrimber(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~SeesawCrimber();
};

#endif /* SeesawCrimber_hpp */
