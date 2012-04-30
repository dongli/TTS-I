#ifndef TestCase_h
#define TestCase_h

#include "MeshManager.hpp"
#include "MeshAdaptor.hpp"
#include "FlowManager.hpp"
#include "TracerManager.hpp"
#include "Coordinate.hpp"

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
