#ifndef TestCase_h
#define TestCase_h

#include "MeshManager.h"
#include "MeshAdaptor.h"
#include "FlowManager.h"
#include "TracerManager.h"
#include "Coordinate.h"

class TestCase
{
public:
    TestCase() {}
    virtual ~TestCase() {}

    virtual void calcVelocityField(FlowManager &flowManager) {}

    virtual void calcInitCond(MeshManager &meshManager,
                              MeshAdaptor &meshAdaptor,
                              TracerManager &tracerManager) {}
};

#endif
