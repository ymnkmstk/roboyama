//
//  utility.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/07/05.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "utility.hpp"

void rgb_to_hsv(rgb_raw_t rgb, hsv_raw_t& hsv) {
    uint16_t max, min;
    double cr, cg, cb, h;  // must be double

    max = rgb.r;
    if(max < rgb.g) max = rgb.g;
    if(max < rgb.b) max = rgb.b;
    
    min = rgb.r;
    if(min > rgb.g) min = rgb.g;
    if(min > rgb.b) min = rgb.b;
    
    hsv.v = 100 * max / (double)255.0;
    
    if (!max) {
        hsv.s = 0;
        hsv.h = 0;
    } else {
        hsv.s = 100 * (max - min) / (double)max;
        cr = (max - rgb.r) / (double)(max - min);
        cg = (max - rgb.g) / (double)(max - min);
        cb = (max - rgb.b) / (double)(max - min);
        
        if (max == rgb.r) {
            h = cb - cg;
        } else if (max == rgb.g) {
            h = 2 + cr - cb;
        } else {
            h = 4 + cg - cr;
        }
        h *= 60;
        if (h < 0) h += 360;
        hsv.h = h;
    }
}
