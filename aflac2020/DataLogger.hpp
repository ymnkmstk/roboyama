//
// DataLogger.hpp
//
// Created by Junsei Sugino on 2020
// Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#define LOGGING_PERIOD ((PERIOD_TRACE_MSG)/(PERIOD_OBS_TSK))

class DataLogger {
private:
    const char *varname;
    int32_t offset;
    int32_t latest;
    char hist_str[2][LOGGING_PERIOD+1];
    int index;
    int count;
public:
    DataLogger( const char *varnm, int32_t offs );
    void logging( int32_t value );
};
