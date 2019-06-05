//
//  app.cpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/04/28.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#include "app.h"

#include "crew.hpp"

Clock*      clock;
Captain*    captain;
Observer*   observer;
Navigator*  activeNavigator = NULL;

// a cyclic handler to activate a task
void task_activator(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    //assert(ercd == E_OK);
}

// Captain's periodic task
void captain_task(intptr_t unused) {
    if (captain != NULL) captain->operate();
}

// Observer's periodic task
void observer_task(intptr_t unused) {
    if (observer != NULL) observer->operate();
}

// Navigator's periodic task
void navigator_task(intptr_t unused) {
    if (activeNavigator != NULL) activeNavigator->operate();
}

void main_task(intptr_t unused) {
    clock   = new Clock;
    captain = new Captain;
    
    captain->takeoff();
    
    // sleep until being waken up
    ER ercd = slp_tsk();
    assert(ercd == E_OK);

    captain->land();
    
    delete captain;
    ext_tsk();
}
