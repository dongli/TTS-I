#ifndef _TestCase_h_
#define _TestCase_h_

#include "FlowManager.h"

class TestCase
{
public:
    TestCase() {}
    virtual ~TestCase() {}

    virtual void calcVelocityField(FlowManager &flowManager) {}
};

#endif
