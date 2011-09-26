#ifndef SolidRotation_h
#define SolidRotation_h

#include "TestCase.h"

class SolidRotation : public TestCase
{
public:
    SolidRotation();
    virtual ~SolidRotation();

    void calcVelocityField(FlowManager &flowManager);

private:
    double U0;
    double alpha;
    Coordinate axisPole;
};

#endif
