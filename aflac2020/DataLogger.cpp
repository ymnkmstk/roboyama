#include "app.h"
#include "DataLogger.hpp"

DataLogger::DataLogger( int32_t offs )
{
    offset = offs;
    latest = 0;
    count = 0;
    prevptr = NULL;
}

//const char *BASE64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void DataLogger::logging( int32_t value )
{
    if ( prevptr == NULL ) {
	latest = value;
	prevptr = hist_str;
    }
    int32_t diff = value - latest;
    diff += (0x22 + offset);
    hist_str[count] = ( diff < 0x21 ) ? 0x21 : ( diff > 0x7e ) ? 0x7e : diff;
    latest = value;
    ++count;
    if ( count >= HISTARRAYSIZE*2 ) count = 0;
}

int DataLogger::getCount()
{
    return (prevptr+count) - hist_str;
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
