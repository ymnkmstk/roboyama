//
//  BlindRunner.hpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#ifndef BlindRunner_hpp
#define BlindRunner_hpp

#include "aflac_common.hpp"
#include "LineTracer.hpp"

#define PROP_NAME_LEN	48	// プロパティー名の最大長
#define NUM_PROPS	13	// プロパティーの個数

struct courseSection {
	char	id[5];
	int32_t sectionEnd;
	double  curvature;
};

const struct courseSection courseMap[] = {
	{"st00",  875, 0.00},
	{"cv01", 1910, 0.55},
	{"st02", 2455, 0.00},
	{"cv03", 3590,-0.48},
	{"st04", 4375, 0.00},
	{"cv05", 4780,-0.70},
	{"st06", 5540, 0.00},
	{"cv07", 6200, 0.50},
	{"st08", 6420, 0.00},
	{"cv09", 7120, 0.46}, //
	{"st10", 7760, 0.00},
	{"cv11", 8360, 0.48},
	{"st12",10160, 0.00},
	{"cv13",10590,-0.70},
	{"st14",11660, 0.00}
}; // Note: size of this array is given by sizeof(courseMap)/sizeof(*courseMap)

class BlindRunner : public LineTracer {
private:
	int16_t s_trace_counter;
    Motor*	leftMotor;
    Motor*  rightMotor;
    Motor*	tailMotor;
	int		courseMapSize, currentSection;

	struct property{
		char name[PROP_NAME_LEN];
		int value;
	};
	struct property props[NUM_PROPS];

	int readLine( FILE* file, char* dst, size_t len );
	void readPropFile( const char* filename );
	int getProp( const char* propname );
protected:
public:
    BlindRunner();
    BlindRunner(Motor* lm, Motor* rm, Motor* tm);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~BlindRunner();
};

#endif /* BlindRunner_hpp */