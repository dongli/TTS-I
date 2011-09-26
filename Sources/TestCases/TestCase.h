#ifndef TestCase_h
#define TestCase_h

#include "FlowManager.h"
#include "TracerManager.h"
#include "Coordinate.h"

class TestCase
{
public:
    TestCase() {}
    virtual ~TestCase() {}

    virtual void calcVelocityField(FlowManager &flowManager) {}

    virtual void calcInitCond(TracerManager &tracerManager) {}
};

#endif
