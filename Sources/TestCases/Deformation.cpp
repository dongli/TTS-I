#include "Deformation.h"
#include "Sphere.h"

Deformation::Deformation(SubCaseID subCaseID, InitCondID initCondID)
{
    this->subCaseID = subCaseID;
    this->initCondID = initCondID;
    T = 5.0;
}

Deformation::~Deformation()
{
}

void Deformation::calcVelocityField(FlowManager &flowManager)
{
    const RLLMesh *umesh = flowManager.u.mesh;
    const RLLMesh *vmesh = flowManager.v.mesh;
    double u[umesh->getNumLon()-2][umesh->getNumLat()][1];
    double v[vmesh->getNumLon()-1][vmesh->getNumLat()][1];
    //                          ^ Note the dimension size!
    double cosT = cos(PI*TimeManager::getSeconds()/T);
    double k;
    Coordinate x;

    // -------------------------------------------------------------------------
    // normal regions
    switch (subCaseID) {
        case Case1:
            k = 2.4;
            for (int i = 1; i < umesh->getNumLon()-1; ++i)
                for (int j = 0; j < umesh->getNumLat(); ++j) {
                    x.set(umesh->lon(i), umesh->lat(j));
                    u[i-1][j][0] = k*pow(sin(x.getLon()*0.5), 2.0)*
                    sin(x.getLat()*2.0)*cosT;
                }
            for (int i = 0; i < vmesh->getNumLon()-1; ++i)
                for (int j = 0; j < vmesh->getNumLat(); ++j) {
                    x.set(vmesh->lon(i), vmesh->lat(j));
                    v[i][j][0] = k*0.5*sin(x.getLon())*cos(x.getLat())*cosT;
                }
            break;
        case Case2:
            k = 2.0;
            for (int i = 1; i < umesh->getNumLon()-1; ++i)
                for (int j = 0; j < umesh->getNumLat(); ++j) {
                    x.set(umesh->lon(i), umesh->lat(j));
                    u[i-1][j][0] = k*pow(sin(x.getLon()), 2.0)*
                    sin(x.getLat()*2.0)*cosT;
                }
            for (int i = 0; i < vmesh->getNumLon()-1; ++i)
                for (int j = 0; j < vmesh->getNumLat(); ++j) {
                    x.set(vmesh->lon(i), vmesh->lat(j));
                    v[i][j][0] = k*sin(x.getLon()*2.0)*cos(x.getLat())*cosT;
                }
            break;
        case Case3:
            k = 1.0;
            for (int i = 1; i < umesh->getNumLon()-1; ++i)
                for (int j = 0; j < umesh->getNumLat(); ++j) {
                    x.set(umesh->lon(i), umesh->lat(j));
                    u[i-1][j][0] = -k*pow(sin(x.getLon()*0.5), 2.0)*
                    sin(x.getLat()*2.0)*pow(cos(x.getLat()), 2.0)*cosT;
                }
            for (int i = 0; i < vmesh->getNumLon()-1; ++i)
                for (int j = 0; j < vmesh->getNumLat(); ++j) {
                    x.set(vmesh->lon(i), vmesh->lat(j));
                    v[i][j][0] = k*0.5*sin(x.getLon())*
                    pow(cos(x.getLat()), 3.0)*cosT;
                }
            break;
        case Case4:
            k = 2.0;
            double factor = PI2*Sphere::radius/T;
            for (int i = 1; i < umesh->getNumLon()-1; ++i)
                for (int j = 0; j < umesh->getNumLat(); ++j) {
                    x.set(umesh->lon(i), umesh->lat(j));
                    u[i-1][j][0] = k*pow(sin(x.getLon()), 2.0)*
                    sin(x.getLat()*2.0)*cosT+factor*cos(x.getLat());
                }
            for (int i = 0; i < vmesh->getNumLon()-1; ++i)
                for (int j = 0; j < vmesh->getNumLat(); ++j) {
                    x.set(vmesh->lon(i), vmesh->lat(j));
                    v[i][j][0] = k*sin(x.getLon()*2.0)*cos(x.getLat())*cosT;
                }
            break;
    }
    // -------------------------------------------------------------------------
    flowManager.update(&u[0][0][0], &v[0][0][0]);
}

void Deformation::calcInitCond(TracerManager &tracerManager)
{
}