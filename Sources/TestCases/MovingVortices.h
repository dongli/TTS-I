#ifndef _MovingVortices_h_
#define _MovingVortices_h_

#include "TestCase.h"
#include "Coordinate.h"

class MovingVortices : public TestCase
{
public:
    MovingVortices();
    virtual ~MovingVortices();

    void calcVelocityField(FlowManager &flowManager);

private:
    void calcVelocityField(FlowManager &flowManager, double time);

    double rho(double lat) const;
    double omega(double latR) const;

    double U0;
    double alpha;
    double rho0;
    Coordinate axisPole;
    Coordinate vortexPos0;
    Coordinate vortexRotPos0;

    double gamma;
};

#endif
