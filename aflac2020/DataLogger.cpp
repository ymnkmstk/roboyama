#include "app.h"
#include "aflac_common.hpp"
#include "DataLogger.hpp"

DataLogger::DataLogger( const char *varnm, int32_t offs )
{
    varname = varnm;
    offset = offs;
    latest = 0;
    index = 0;
    count = 0;
}

//const char *BASE64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void DataLogger::logging( int32_t value )
{
    int32_t diff = value - latest;
    diff += (0x22 + offset);
    hist_str[index][count] = ( diff < 0x21 ) ? 0x21 : ( diff > 0x7e ) ? 0x7e : diff;
    latest = value;
    ++count;
    if ( count >= LOGGING_PERIOD ) {
	char *ptr = hist_str[index];
	count = 0;
	index = 1-index;
	_debug(syslog(LOG_NOTICE, "%08u, DataLogger:%s = %d %s",
		      clock->now(), varname, value, ptr));
    }
}
