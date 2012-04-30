#ifndef _StaticVortices_h_
#define _StaticVortices_h_

#include "TestCase.hpp"

class StaticVortices : public TestCase
{
public:
    StaticVortices();
    virtual ~StaticVortices();

    void calcVelocityField(FlowManager &flowManager);

private:
    double rho(double lat) const;
    double omega(double latR) const;

    double T0;
    double U0;
    double alpha;
    double rho0;
    Coordinate axisPole;
    Coordinate vortexPos;
};

#endif
