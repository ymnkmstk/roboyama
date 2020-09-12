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
	char	id[6];
	int32_t sectionEnd;
	double  curvature;
};

// section id starts with L uses LineTracer
// while   id starts with B uses BlindRunner logic
// Note:
//  curve +625, 0.45 makes right angle 
const struct courseSection courseMap[] = {
	{"Lst00",  855, 0.0},
	{"Bcv01", 1370, 0.533},
	{"Bcv01", 1925, 0.5335},
	{"Lst02", 2425, 0.0},
	{"Bcv03", 3610,-0.4795},
	{"Lst04", 4213, 0.0},    
 	{"Bcv05", 4838,-0.45},
	{"Lst06", 5400, 0.0},
	{"Bcv07", 6025, 0.45},
	{"Bst08", 6310, 0.0},
	{"Bcv09", 6925, 0.45},
	{"Lst10", 7455, 0.0},
	{"Bcv11", 8080, 0.45},
	{"Bst12", 9000, 0.0},
	{"Bst12",10160, 0.0},
	{"Lcv13",10590,-0.70},
	{"Lst14",11660, 0.0}
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