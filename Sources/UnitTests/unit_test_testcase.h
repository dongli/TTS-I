#ifndef unit_test_testcase_h
#define unit_test_testcase_h

#include "MovingVortices.h"
#include "SolidRotation.h"
#include "Deformation.h"
#include "TimeManager.h"

template <class TESTCASE>
void test_TestCase(double timeStep, int steps)
{
    VelocityField2D velocityField;
    EvalWindField2D evalWindField;
    TESTCASE testCase;
    
    TimeManager::setClock(timeStep);
    TimeManager::setEndStep(steps);
    
    while (!TimeManager::isFinished()) {
        testCase.calcVelocityField(velocityField.flowManager);
        velocityField.flowManager.output("flow.nc");
        for (int j = 0; j < evalWindField.numLat; ++j)
            for (int i = 0; i < evalWindField.numLon; ++i) {
                Location loc;
                Coordinate x;
                Velocity velocity;
                x.setSPH(evalWindField.lon[i], evalWindField.lat[j]);
                velocityField.meshManager.checkLocation(x, loc);
                velocityField.flowManager.getVelocity(x, loc, NewTimeLevel,
                                                      velocity, Velocity::LonLatSpace);
                evalWindField.u(j, i) = velocity.u;
                evalWindField.v(j, i) = velocity.v;
            }
        evalWindField.output("test_flow.nc");
        TimeManager::advance();
    }
}

#endif