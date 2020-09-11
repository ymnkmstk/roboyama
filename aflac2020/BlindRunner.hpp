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
	{"st00",  855, 0.0},
	{"cv01", 1910, 0.5333},
	{"st02", 2428, 0.0},
	{"cv03", 3570,-0.4795},
	{"st04", 4355, 0.0},
	{"cv05", 4780,-0.75},
	{"st06", 5535, 0.0},
	{"cv07", 6190, 0.488}, //
	{"st08", 6420, 0.0},
	{"cv09", 7120, 0.46},
	{"st10", 7760, 0.0},
	{"cv11", 8360, 0.48},
	{"st12",10160, 0.0},
	{"cv13",10590,-0.70},
	{"st14",11660, 0.0}
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