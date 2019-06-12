//
//  LimboDancer.hpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/06/12.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef LimboDancer_hpp
#define LimboDancer_hpp

#include "crew.hpp"

class LimboDancer : public Navigator {
protected:
public:
    LimboDancer();
    LimboDancer(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~LimboDancer();
};

#endif /* LimboDancer_hpp */
