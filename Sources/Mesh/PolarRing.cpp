#include "Sphere.h"
#include "Constants.h"
#include "ReportMacros.h"
#include "TimeManager.h"
#include "PolarRing.h"

PolarRingVelocity::PolarRingVelocity()
{
    isInitialized = false;
}

PolarRingVelocity::~PolarRingVelocity()
{
}

void PolarRingVelocity::linkVelocityField(const Field &u, const Field &v)
{
    this->uField = &u;
    this->vField = &v;
    int numLon, numLev;
    numLon = v.mesh->getNumLon()-1;
    if (v.layers != NULL) {
        numLev = v.layers->getNumLev();
    } else {
        numLev = 1;
    }
    lon.resize(numLon);
    sinLon.resize(numLon);
    cosLon.resize(numLon);
    for (int i = 0; i < numLon; ++i) {
        lon[i] = v.mesh->lon(i);
        sinLon[i] = sin(lon[i]);
        cosLon[i] = cos(lon[i]);
    }
    lat[0] = u.mesh->lat(0);
    lat[1] = u.mesh->lat(u.mesh->lat.size()-1);
    for (int j = 0; j < 2; ++j) {
        sinLat[j] = sin(lat[j]);
        sinLat2[j] = sinLat[j]*sinLat[j];
        this->u[j].resize(numLon, numLev);
        this->v[j].resize(numLon, numLev);
        this->ut[j].resize(numLon, numLev);
        this->vt[j].resize(numLon, numLev);
        for (int i = 0; i < numLon; ++i)
            for (int k = 0; k < numLev; ++k) {
                this->u[j](i, k).init();
                this->v[j](i, k).init();
                this->ut[j](i, k).init();
                this->vt[j](i, k).init();
            }
    }
}

void PolarRingVelocity::update()
{
    if (! isInitialized) {
        for (int j = 0; j < 2; ++j) {
            int l = j == 0 ? 0 : uField->mesh->getNumLat()-1;
            for (int i = 0; i < u[j].extent(0); ++i)
                for (int k = 0; k < u[j].extent(1); ++k) {
                    u[j](i, k).setNew((uField->values(i, l, k).getNew()+
                        uField->values(i+1, l, k).getNew())*0.5);
                    u[j](i, k).save();
                }
        }
        for (int j = 0; j < 2; ++j) {
            int l = j == 0 ? 0 : vField->mesh->getNumLat()-2;
            for (int i = 0; i < v[j].extent(0); ++i)
                for (int k = 0; k < v[j].extent(1); ++k) {
                    v[j](i, k).setNew((vField->values(i, l, k).getNew()+
                        vField->values(i, l+1, k).getNew())*0.5);
                    v[j](i, k).save();
                }
        }
        // transform velocity
        double tmp1, tmp2;
        for (int j = 0; j < 2; ++j) {
            double sign = j == 0 ? 1.0 : -1.0;
            for (int i = 0; i < u[j].extent(0); ++i)
                for (int k = 0; k < u[j].extent(1); ++k) {
                    tmp1 = -sign*sinLon[i]/sinLat[j]*u[j](i, k).getNew();
                    tmp2 = -sign*cosLon[i]/sinLat2[j]*v[j](i, k).getNew();
                    ut[j](i, k).setNew(tmp1+tmp2);
                    ut[j](i, k).save();
                    tmp1 = sign*cosLon[i]/sinLat[j]*u[j](i, k).getNew();
                    tmp2 = -sign*sinLon[i]/sinLat2[j]*v[j](i, k).getNew();
                    vt[j](i, k).setNew(tmp1+tmp2);
                    vt[j](i, k).save();
                }
        }
        isInitialized = true;
    } else {
        for (int j = 0; j < 2; ++j) {
            int l = j == 0 ? 0 : uField->mesh->getNumLat()-1;
            for (int i = 0; i < u[j].extent(0); ++i)
                for (int k = 0; k < u[j].extent(1); ++k) {
                    u[j](i, k).save();
                    u[j](i, k).setNew((uField->values(i, l, k).getNew()+
                        uField->values(i+1, l, k).getNew())*0.5);
                }
        }
        for (int j = 0; j < 2; ++j) {
            int l = j == 0 ? 0 : vField->mesh->getNumLat()-2;
            for (int i = 0; i < v[j].extent(0); ++i)
                for (int k = 0; k < v[j].extent(1); ++k) {
                    v[j](i, k).save();
                    v[j](i, k).setNew((vField->values(i, l, k).getNew()+
                        vField->values(i, l+1, k).getNew())*0.5);
                }
        }
        // transform velocity
        double tmp1, tmp2;
        for (int j = 0; j < 2; ++j) {
            double sign = j == 0 ? 1.0 : -1.0;
            for (int i = 0; i < u[j].extent(0); ++i)
                for (int k = 0; k < u[j].extent(1); ++k) {
                    tmp1 = -sign*sinLon[i]/sinLat[j]*u[j](i, k).getNew();
                    tmp2 = -sign*cosLon[i]/sinLat2[j]*v[j](i, k).getNew();
                    ut[j](i, k).save();
                    ut[j](i, k).setNew(tmp1+tmp2);
                    tmp1 = sign*cosLon[i]/sinLat[j]*u[j](i, k).getNew();
                    tmp2 = -sign*sinLon[i]/sinLat2[j]*v[j](i, k).getNew();
                    vt[j](i, k).save();
                    vt[j](i, k).setNew(tmp1+tmp2);
                }
        }

    }
}

Velocity PolarRingVelocity::interp(const Coordinate &x, const Location &loc,
                                   TimeLevel timeLevel) const
{
    Velocity velocity;
    velocity.ut = 0.0; velocity.vt = 0.0;
    double weightSum = 0.0;
    const double eps = 1.0e-10;
    Coordinate x1;
    double distance;
    for (int i = 0; i < getNumLon(); ++i) {
        x1.set(getLon(i), getLat(loc.pole), x.getLev());
        distance = Sphere::calcDistance(x1, x);
        if (distance < eps) {
            velocity.ut = ut[loc.pole](i, loc.k).get(timeLevel);
            velocity.vt = vt[loc.pole](i, loc.k).get(timeLevel);
            velocity.u = u[loc.pole](i, loc.k).get(timeLevel);
            velocity.v = v[loc.pole](i, loc.k).get(timeLevel);
            return velocity;
        } else {
            double weight = 1.0/distance/distance;
            weightSum += weight;
            velocity.ut += weight*ut[loc.pole](i, loc.k).get(timeLevel);
            velocity.vt += weight*vt[loc.pole](i, loc.k).get(timeLevel);
        }
    }
    velocity.ut /= weightSum;
    velocity.vt /= weightSum;
    return velocity;
}
