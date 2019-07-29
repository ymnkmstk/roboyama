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

#define TAIL_ANGLE_LIMBO      60  /* リンボーダンス時の角度[度] */

class LimboDancer : public LineTracer {
protected:
public:
    LimboDancer();
    LimboDancer(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~LimboDancer();
    int16_t limboMode;
    int16_t counter;
};

#endif /* LimboDancer_hpp */
