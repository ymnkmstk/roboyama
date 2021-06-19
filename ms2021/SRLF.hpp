/*
    SRLF.hpp
    Srew Rate Limiter Filter
    for trapezoidal motion

    Copyright © 2021 MS Mode 2. All rights reserved.
*/
#ifndef SRLF_hpp
#define SRLF_hpp

#include "Filter.hpp"

class SRLF : public Filter {
public:
    SRLF(const double rate);
    double setRate(const double rate);
    inline double apply(const double xin);
protected:
    double srewRate;
    double prevXin;
};

inline double SRLF::apply(const double xin) {
    double delta = xin - prevXin;
    if (srewRate < delta) {
        delta = srewRate;
    }
    if (-srewRate > delta) {
        delta = -srewRate;
    }
    prevXin += srewRate;
    return prevXin;
}

#endif /* SRLF_hpp */