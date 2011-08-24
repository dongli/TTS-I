#include "FlowManager.h"
#include "TimeManager.h"
#include "ReportMacros.h"
#include "Sphere.h"
#include "Constants.h"
#include <netcdfcpp.h>
#include <sys/stat.h>

FlowManager::FlowManager()
{
    isInitialized = false;
#ifndef UNIT_TEST
    REPORT_ONLINE("FlowManager")
#endif
}

FlowManager::~FlowManager()
{
#ifndef UNIT_TEST
    REPORT_OFFLINE("FlowManager")
#endif
}

void FlowManager::construct(const MeshManager &meshManager)
{
    u.name = "u";
    u.long_name = "Zonal velocity";
    u.unit = "m s-1";

    v.name = "v";
    v.long_name = "Meridinal velocity";
    v.unit = "m s-1";

    if (meshManager.hasLayers()) {
        u.construct(meshManager.mesh[RLLMesh::LonHalf],
                    meshManager.layers[Layers::Full]);
        v.construct(meshManager.mesh[RLLMesh::LatHalf],
                    meshManager.layers[Layers::Full]);

        w.name = "w";
        w.long_name = "Vertical velocity";
        w.unit = "Unknown";
        w.construct(meshManager.mesh[RLLMesh::Full],
                    meshManager.layers[Layers::Half]);
    } else {
        u.construct(meshManager.mesh[RLLMesh::LonHalf]);
        v.construct(meshManager.mesh[RLLMesh::LatHalf]);
    }
    prv.linkVelocityField(u, v);
}

void FlowManager::update(double *u, double *v)
{
    int numLev = this->u.layers == NULL ? 1 : this->u.layers->getNumLev();
    // Note:
    //   (1) The multi-dimensional u and v have been represesnted as
    //       one dimensional array here. The order of storage should
    //       be considered carefully (first latitude grids, then longitude);
    //   (2) The first-step-setting can affect the restart.
    int l;
    if (! isInitialized) {
        l = -1;
        for (int i = 1; i < this->u.mesh->getNumLon()-1; ++i)
            for (int j = 0; j < this->u.mesh->getNumLat(); ++j)
                for (int k = 0; k < numLev; ++k) {
                    this->u.values(i, j, k).setNew(u[++l]);
                    this->u.values(i, j, k).save();
                }
        l = -1;
        for (int i = 0; i < this->v.mesh->getNumLon()-1; ++i)
            for (int j = 0; j < this->v.mesh->getNumLat(); ++j)
                for (int k = 0; k < numLev; ++k) {
                    this->v.values(i, j, k).setNew(v[++l]);
                    this->v.values(i, j, k).save();
                }
        isInitialized = true;
    } else {
        l = -1;
        for (int i = 1; i < this->u.mesh->getNumLon()-1; ++i)
            for (int j = 0; j < this->u.mesh->getNumLat(); ++j)
                for (int k = 0; k < numLev; ++k) {
                    this->u.values(i, j, k).save();
                    this->u.values(i, j, k).setNew(u[++l]);
                }
        l = -1;
        for (int i = 0; i < this->v.mesh->getNumLon()-1; ++i)
            for (int j = 0; j < this->v.mesh->getNumLat(); ++j)
                for (int k = 0; k < numLev; ++k) {
                    this->v.values(i, j, k).save();
                    this->v.values(i, j, k).setNew(v[++l]);
                }
    }
    prv.update();
}

void FlowManager::getVelocity(const Coordinate &x, const Location &loc,
                              TimeLevel timeLevel, Velocity &velocity,
                              Velocity::Type type) const
{
    if (loc.inPolarCap) {
        velocity = prv.interp(x, loc, timeLevel);
        if (type == Velocity::LonLatSpace) {
            double sign = loc.pole == Location::NorthPole ? 1.0 : -1.0;
            double sinLon = sin(x.getLon());
            double cosLon = cos(x.getLon());
            double sinLat = sin(x.getLat());
            double sinLat2 = sinLat*sinLat;
            velocity.u = sign*(-sinLon*velocity.ut+cosLon*velocity.vt)*sinLat;
            velocity.v = sign*(-cosLon*velocity.ut-sinLon*velocity.vt)*sinLat2;
        }
    } else {
        velocity.u = u.interp(x, loc, timeLevel);
        velocity.v = v.interp(x, loc, timeLevel);
        if (type == Velocity::StereoPlane) {
            // Note: Here we use the positiveness of latitude to judge the point
            //       is in north pole or south pole, NOT from the loc.pole.
            double sign = x.getLat() > 0.0 ? 1.0 : -1.0;
            double sinLon = sin(x.getLon());
            double cosLon = cos(x.getLon());
            double sinLat = sin(x.getLat());
            double sinLat2 = sinLat*sinLat;
            velocity.ut = sign*(-sinLon/sinLat*velocity.u-cosLon/sinLat2*velocity.v);
            velocity.vt = sign*( cosLon/sinLat*velocity.u-sinLon/sinLat2*velocity.v);
        }
    }
}

void FlowManager::output(const string &fileName) const
{
    NcFile *file;

    struct stat statInfo;
    int ret = stat(fileName.c_str(), &statInfo);

    if (ret != 0) {
        file = new NcFile(fileName.c_str(), NcFile::Replace);
    } else {
        file = new NcFile(fileName.c_str(), NcFile::Write);
    }

    if (!file->is_valid()) {
        char message[100];
        sprintf(message, "Failed to open file %s.", fileName.c_str());
        REPORT_ERROR(message)
    }

    NcError ncError(NcError::silent_nonfatal);

    int numLon = v.mesh->getNumLon()-1;
    int numLat = u.mesh->getNumLat();

    NcVar *timeVar, *uVar, *vVar;

    if (ret != 0) {
        NcDim *timeDim = file->add_dim("time");
        NcDim *lonDim = file->add_dim("lon", numLon);
        NcDim *latDim = file->add_dim("lat", numLat);

        timeVar = file->add_var("time", ncDouble, timeDim);
        NcVar *lonVar = file->add_var("lon", ncDouble, lonDim);
        NcVar *latVar = file->add_var("lat", ncDouble, latDim);

        lonVar->add_att("long_name", "longitude");
        lonVar->add_att("units", "degree_east");
        latVar->add_att("long_name", "latitude");
        latVar->add_att("units", "degree_north");

        double lon[numLon];
        double lat[numLat];
        for (int i = 0; i < numLon; ++i)
            lon[i] = v.mesh->lon(i)*Rad2Deg;
        for (int j = 0; j < numLat; ++j)
            lat[j] = u.mesh->lat(j)*Rad2Deg;
        lonVar->put(lon, numLon);
        latVar->put(lat, numLat);

        uVar = file->add_var(u.name.c_str(), ncDouble, timeDim, latDim, lonDim);
        vVar = file->add_var(v.name.c_str(), ncDouble, timeDim, latDim, lonDim);

        uVar->add_att("long_name", u.long_name.c_str());
        uVar->add_att("units", u.unit.c_str());
        vVar->add_att("long_name", v.long_name.c_str());
        vVar->add_att("units", v.unit.c_str());
    } else {
        timeVar = file->get_var("time");
        uVar = file->get_var(u.long_name.c_str());
        vVar = file->get_var(v.long_name.c_str());
    }

    double seconds = TimeManager::getSeconds();
    int record = TimeManager::getSteps();
    timeVar->put_rec(&seconds, record);

    double uu[numLat][numLon];
    double vv[numLat][numLon];
    for (int j = 0; j < numLat; ++j)
        for (int i = 0; i < numLon; ++i) {
            uu[j][i] = (u.values(i, j).getNew()+u.values(i+1, j).getNew())*0.5;
            vv[j][i] = (v.values(i, j).getNew()+v.values(i+1, j).getNew())*0.5;
        }
    uVar->put_rec(&uu[0][0], record);
    vVar->put_rec(&vv[0][0], record);

    file->close();

    delete file;
}
