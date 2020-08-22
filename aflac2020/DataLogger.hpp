//
// DataLogger.hpp
//
// Created by Junsei Sugino on 2020
// Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#define HISTARRAYSIZE (((PERIOD_TRACE_MSG)/(PERIOD_OBS_TSK))+50)

class DataLogger {
private:
    int32_t offset;
    int32_t latest;
    char hist_str[HISTARRAYSIZE*2];
    int count;
    char* prevptr;
public:
    DataLogger( int32_t offs );
    void logging( int32_t value );
    int32_t lastValue();
    char* getHistByString();
};
