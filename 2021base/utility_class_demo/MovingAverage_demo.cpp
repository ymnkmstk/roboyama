// this example demonstrates how MovingAverage class works
//
// g++ -std=c++11 MovingAverage_demo.cpp && ./a.out
#include <iostream>
using namespace std;
#include "../MovingAverage.hpp"

#define CAP 7

int main() {
    MovingAverage<double, CAP> *ma = new MovingAverage<double, CAP>();
    cout << "capacity = " << CAP << endl;
    cout << " ave = " << ma->push(32) << ", stdev = " << ma->stdev() << endl;
    cout << " ave = " << ma->push(47) << ", stdev = " << ma->stdev() << endl;
    cout << " ave = " << ma->push(42) << ", stdev = " << ma->stdev() << endl;
    cout << " ave = " << ma->push(45) << ", stdev = " << ma->stdev() << endl;
    cout << " ave = " << ma->push(80) << ", stdev = " << ma->stdev() << endl;
    cout << " ave = " << ma->push(90) << ", stdev = " << ma->stdev() << endl;
    cout << " ave = " << ma->push(52) << ", stdev = " << ma->stdev() << endl;
    cout << " ave = " << ma->push(32) << ", stdev = " << ma->stdev() << endl;
    return 0;
}