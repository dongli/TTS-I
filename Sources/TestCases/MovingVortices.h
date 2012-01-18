#ifndef _MovingVortices_h_
#define _MovingVortices_h_

#include "TestCase.h"

class MovingVortices : public TestCase
{
public:
    MovingVortices();
    virtual ~MovingVortices();

    void calcVelocityField(FlowManager &);

    void calcInitCond(MeshManager &, MeshAdaptor &, TracerManager &);

    void calcSolution(double time, const Array<double, 1> &lon,
                      const Array<double, 1> &lat, Array<double, 2> &q);

private:
    void calcVelocityField(FlowManager &flowManager, double time);

    double rho(double lat) const;
    double omega(double latR) const;

    double U0;
    double alpha;
    double rho0;
    Coordinate axisPole;

    double gamma;
    Coordinate xv0; // initial vortex coordinate
    Coordinate xvr0; // rotated initial vortex coordinate
};

#endif
