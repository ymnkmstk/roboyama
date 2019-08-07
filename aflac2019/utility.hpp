//
//  utility.hpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/07/05.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef utility_hpp
#define utility_hpp

#include <cinttypes>
#include "ColorSensor.h"
using namespace ev3api;

typedef struct {
    uint16_t h; // Hue
    uint16_t s; // Saturation
    uint16_t v; // Value of brightness
    
} hsv_raw_t;

void rgb_to_hsv(rgb_raw_t rgb, hsv_raw_t& hsv);

#endif /* utility_hpp */
