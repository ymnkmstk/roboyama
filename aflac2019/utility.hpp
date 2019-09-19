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

template<int ORDER> class FIR_Direct {
private:
    const double *const hm;
    double un[ORDER+1];
public:
    FIR_Direct(const double hk[]);
    inline double Execute(const double xin);
};

template<int ORDER>
FIR_Direct<ORDER>::FIR_Direct(const double hk[]) : hm(hk) {
    for (int i = 0; i <= ORDER; i++) un[i] = 0.0;
}

template<int ORDER>
inline double FIR_Direct<ORDER>::Execute(const double xin) {
    double acc = 0.0;
    un[0] = xin;
    for (int i = 0; i <= ORDER; i++) acc = acc + hm[i] * un[i];
    for (int i = ORDER; i > 0; i--) un[i] = un[i-1];
    return acc;
}

template<int ORDER> class FIR_Transposed {
private:
    const double *const hm;
    double un[ORDER+1];
public:
    FIR_Transposed(const double hk[]);
    inline double Execute(const double xin);
};

template<int ORDER>
FIR_Transposed<ORDER>::FIR_Transposed(const double hk[]) : hm(hk) {
    for (int i = 0; i <= ORDER; i++) un[i] = 0.0;
}

template<int ORDER>
inline double FIR_Transposed<ORDER>::Execute(const double xin) {
    for (int i = 0; i < ORDER; i++) un[i] = hm[i] * xin + un[i+1];
    un[ORDER] = hm[ORDER] * xin;
    return un[0];
}

void rgb_to_hsv(rgb_raw_t rgb, hsv_raw_t& hsv);

class PIDcalculator {
private:
    long double kp, ki, kd;   /* PID constant */
    int16_t diff[2], deltaT, minimum, maximum;
    long double integral;
    int16_t math_limit(int16_t input, int16_t min, int16_t max);
public:
    PIDcalculator(long double p, long double i, long double d, int16_t t, int16_t min, int16_t max);
    int16_t compute(int16_t sensor, int16_t target);
    ~PIDcalculator();
};

#endif /* utility_hpp */
