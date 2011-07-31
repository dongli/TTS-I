#include "SolidRotation.h"
#include "TimeManager.h"
#include "Constants.h"
#include "ReportMacros.h"
#include "Sphere.h"

SolidRotation::SolidRotation()
{
    U0 = PI2*Sphere::radius/12.0/86400.0;
    alpha = PI05;
    axisPole.set(PI, PI05-alpha);
}

SolidRotation::~SolidRotation()
{
}

void SolidRotation::calcVelocityField(FlowManager &flowManager)
{
    const RLLMesh *umesh = flowManager.u.mesh;
    const RLLMesh *vmesh = flowManager.v.mesh;
    double u[umesh->getNumLon()-2][umesh->getNumLat()][1];
    double v[vmesh->getNumLon()-1][vmesh->getNumLat()][1];
    //                          ^ Note the dimension size!
    
    // -------------------------------------------------------------------------
    // normal regions
    for (int i = 1; i < umesh->getNumLon()-1; ++i)
        for (int j = 0; j < umesh->getNumLat(); ++j) {
            Coordinate x;
            x.set(umesh->lon(i), umesh->lat(j));
            double tmp1, tmp2;
            tmp1 = cos(x.getLat())*cos(alpha);
            tmp2 = sin(x.getLat())*cos(x.getLon())*sin(alpha);
            double rotatePart = U0*(tmp1+tmp2);
            u[i-1][j][0] = rotatePart;
            //  ^ Note the index!
        }

    for (int i = 0; i < vmesh->getNumLon()-1; ++i)
        for (int j = 0; j < vmesh->getNumLat(); ++j) {
            Coordinate x;
            x.set(vmesh->lon(i), vmesh->lat(j));
            double rotatePart = -U0*sin(x.getLon())*sin(alpha);
            v[i][j][0] = rotatePart;
        }
    
    // -------------------------------------------------------------------------
    flowManager.update(&u[0][0][0], &v[0][0][0]);
}
