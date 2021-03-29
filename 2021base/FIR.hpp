/*
    FIR.hpp
    Finite Impulse Response Filter

    Extracted from the following book:
      C++活用DCPプログラミング
      ASIN: B005FOHWA2
    Copyright © 2006 Naoki Mikami. All rights reserved.
*/

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