#ifndef _SolidRotation_h_
#define _SolidRotation_h_

#include "TestCase.h"
#include "Coordinate.h"

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
