#include "app.h"
#include "DataLogger.hpp"

DataLogger::DataLogger()
{
    latest = 0;
    count = 0;
}

const char *BASE64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void DataLogger::logging( int32_t value )
{
    int32_t diff = value - latest;
    hist_str[count] = ( diff < -32 ) ? '-' : ( 31 < diff ) ? '!' : BASE64[diff & 0x3F];
    latest = value;
    ++count;
}

char* DataLogger::getHistByString()
{
    hist_str[count] = 0;
    if ( count > HISTARRAYSIZE ) {
	count = 0;
	return prevptr;
    } else {
	++count;
	prevptr = hist_str + count;
	return hist_str;
    }
}

int32_t DataLogger::lastValue()
{
    return latest;
}
