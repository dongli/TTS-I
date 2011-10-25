#include "MovingVortices.h"
#include "TimeManager.h"
#include "Constants.h"
#include "ReportMacros.h"
#include "Sphere.h"

MovingVortices::MovingVortices()
{
    U0 = PI2*Sphere::radius/12.0/86400.0;
    alpha = PI05;
    rho0 = 3.0;
    axisPole.set(PI, PI05-alpha);

    gamma = 5.0;
    vortexPos0.set(PI05, 0.0);
    Sphere::rotate(axisPole, vortexPos0, vortexRotPos0);
}

MovingVortices::~MovingVortices()
{
}

void MovingVortices::calcVelocityField(FlowManager &flowManager)
{
    if (TimeManager::getSteps() != 0 && TimeManager::isFirstStep()) {
        // for restarting
        double time = TimeManager::getSeconds()-TimeManager::getTimeStep();
        calcVelocityField(flowManager, time);
    }
    calcVelocityField(flowManager, TimeManager::getSeconds());
}

void MovingVortices::calcVelocityField(FlowManager &flowManager, double time)
{
    Coordinate vortexRotPos;
    double lon = vortexRotPos0.getLon()+U0/Sphere::radius*time;
    if (lon > PI2) lon -= PI2;
    vortexRotPos.set(lon, vortexRotPos0.getLat());
    Coordinate vortexPos;
    Sphere::inverseRotate(axisPole, vortexPos, vortexRotPos);

    const RLLMesh &umesh = flowManager.u.getMesh();
    const RLLMesh &vmesh = flowManager.v.getMesh();
    double u[umesh.getNumLon()-2][umesh.getNumLat()][1];
    double v[vmesh.getNumLon()-1][vmesh.getNumLat()][1];

    // -------------------------------------------------------------------------
    // normal regions
    for (int i = 1; i < umesh.getNumLon()-1; ++i)
        for (int j = 0; j < umesh.getNumLat(); ++j) {
            Coordinate x, xr;
            x.set(umesh.lon(i), umesh.lat(j));
            Sphere::rotate(vortexPos, x, xr);
            double ReOmega = Sphere::radius*omega(xr.getLat());
            double dlon = x.getLon()-vortexPos.getLon();
            double tmp1, tmp2;
            tmp1 = cos(x.getLat())*cos(alpha);
            tmp2 = sin(x.getLat())*cos(x.getLon())*sin(alpha);
            double rotatePart = U0*(tmp1+tmp2);
            tmp1 = sin(vortexPos.getLat())*cos(x.getLat());
            tmp2 = cos(vortexPos.getLat())*cos(dlon)*sin(x.getLat());
            double deformPart = ReOmega*(tmp1-tmp2);
            u[i-1][j][0] = rotatePart+deformPart;
        }

    for (int i = 0; i < vmesh.getNumLon()-1; ++i)
        for (int j = 0; j < vmesh.getNumLat(); ++j) {
            Coordinate x, xr;
            x.set(vmesh.lon(i), vmesh.lat(j));
            Sphere::rotate(vortexPos, x, xr);
            double ReOmega = Sphere::radius*omega(xr.getLat());
            double dlon = x.getLon()-vortexPos.getLon();
            double rotatePart = -U0*sin(x.getLon())*sin(alpha);
            double deformPart = ReOmega*cos(vortexPos.getLat())*sin(dlon);
            v[i][j][0] = rotatePart+deformPart;
        }

    // -------------------------------------------------------------------------
    flowManager.update(&u[0][0][0], &v[0][0][0]);
}

inline double MovingVortices::rho(double lat) const
{
    return rho0*cos(lat);
}

double MovingVortices::omega(double latR) const
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
