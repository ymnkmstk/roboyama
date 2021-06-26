/*
    Logger.hpp
    
    Copyright © 2021 Tomoko Sano / MS Mode 2. All rights reserved.
*/
#ifndef Logger_hpp
#define Logger_hpp

#include "appusr.hpp"

class Logger{
public:
    Logger();
    void outputLog(int logInterval);
protected:
    int traceCnt;
    int32_t prevAngL, prevAngR;
};

#endif /* Logger_hpp */