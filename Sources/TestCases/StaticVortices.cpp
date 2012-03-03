#include "StaticVortices.h"
#include "Constants.h"
#include "ReportMacros.h"
#include "Sphere.h"

StaticVortices::StaticVortices()
{
    T0 = 12.0*86400.0;
    U0 = PI2*Sphere::radius/T0;
    alpha = PI05;
    rho0 = 3.0;
    axisPole.setSPH(PI, PI05-alpha);

    vortexPos.setSPH(0.0, 0.0);
}

StaticVortices::~StaticVortices()
{
}

void StaticVortices::calcVelocityField(FlowManager &flowManager)
{
    const RLLMesh &umesh = flowManager.u.getMesh();
    const RLLMesh &vmesh = flowManager.v.getMesh();
    double u[umesh.getNumLon()-2][umesh.getNumLat()][1];
    double v[vmesh.getNumLon()-1][vmesh.getNumLat()][1];
    double timeRatio = cos(PI*TimeManager::getSeconds()/T0);

    // -------------------------------------------------------------------------
    // normal regions
    for (int i = 1; i < umesh.getNumLon()-1; ++i)
        for (int j = 0; j < umesh.getNumLat(); ++j) {
            Coordinate x, xr;
            x.setSPH(umesh.lon(i), umesh.lat(j));
            Sphere::rotate(vortexPos, x, xr);
            double ReOmega = Sphere::radius*omega(xr.getLat());
            double dlon = x.getLon()-vortexPos.getLon();
            double tmp1, tmp2;
            tmp1 = sin(vortexPos.getLat())*cos(x.getLat());
            tmp2 = cos(vortexPos.getLat())*cos(dlon)*sin(x.getLat());
            double deformPart = ReOmega*(tmp1-tmp2);
            u[i-1][j][0] = deformPart*timeRatio;
        }

    for (int i = 0; i < vmesh.getNumLon()-1; ++i)
        for (int j = 0; j < vmesh.getNumLat(); ++j) {
            Coordinate x, xr;
            x.setSPH(vmesh.lon(i), vmesh.lat(j));
            Sphere::rotate(vortexPos, x, xr);
            double ReOmega = Sphere::radius*omega(xr.getLat());
            double dlon = x.getLon()-vortexPos.getLon();
            double deformPart = ReOmega*cos(vortexPos.getLat())*sin(dlon);
            v[i][j][0] = deformPart*timeRatio;
        }
    // -------------------------------------------------------------------------
    flowManager.update(&u[0][0][0], &v[0][0][0]);
}

inline double StaticVortices::rho(double lat) const
{
    return rho0*cos(lat);
}

double StaticVortices::omega(double latR) const
{
    static const double fac = 1.5*sqrt(3.0);
    double r = rho(latR); // radial distance of vortex
    double U; // Tangential velocity

    if (fabs(r) < 1.0e-12) {
        return 0.0;
    } else {
        U = U0/Sphere::radius*fac*tanh(r)/(cosh(r)*cosh(r));
        return U/r;
    }
}
