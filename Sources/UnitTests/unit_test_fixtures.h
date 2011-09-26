#ifndef unit_test_fixture_h
#define unit_test_fixture_h

#include <sys/stat.h>

#include "MeshManager.h"
#include "FlowManager.h"
#include "TimeManager.h"

class VelocityField2D
{
public:
    VelocityField2D() {
        int numLon = 360, numLat = 180;
        double lon[numLon], lat[numLat];

        double dlon = PI2/numLon;
        double dlat = PI/(numLat+1);

        for (int i = 0; i < numLon; ++i) {
            lon[i] = i*dlon;
        }
        for (int j = 0; j < numLat; ++j) {
            lat[j] = PI*0.5-(j+1)*dlat;
        }
        meshManager.init(numLon, numLat, lon, lat);
        flowManager.init(meshManager);
    }
    ~VelocityField2D() {}

    MeshManager meshManager;
    FlowManager flowManager;
};

class EvalWindField2D
{
public:
    EvalWindField2D() {
        numLon = 256;
        numLat = 360;

        lon = new double[numLon];
        lat = new double[numLat];

        double dlon = PI2/numLon;
        double dlat = PI/(numLat-1);
        
        for (int i = 0; i < numLon; ++i)
            lon[i] = i*dlon;

        for (int j = 0; j < numLat; ++j)
            lat[j] = PI05-j*dlat;

        u.resize(numLat, numLon);
        v.resize(numLat, numLon);
    }
    ~EvalWindField2D() {
        delete [] lon;
        delete [] lat;
    }

    void output(const string &fileName) {
        NcFile *file;
    
        struct stat statInfo;
        int ret = stat(fileName.c_str(), &statInfo);
        
        NcError ncError(NcError::silent_nonfatal);
        
        NcVar *timeVar, *uVar, *vVar;

        if (ret != 0 || TimeManager::isFirstStep()) {
            file = new NcFile(fileName.c_str(), NcFile::Replace);
            
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
            
            double lonDeg[numLon], latDeg[numLat];
            for (int i = 0; i < numLon; ++i)
                lonDeg[i] = lon[i]*Rad2Deg;
            for (int j = 0; j < numLat; ++j)
                latDeg[j] = lat[j]*Rad2Deg;
            lonVar->put(lonDeg, numLon);
            latVar->put(latDeg, numLat);
            
            uVar = file->add_var("u", ncDouble, timeDim, latDim, lonDim);
            vVar = file->add_var("v", ncDouble, timeDim, latDim, lonDim);
            
            uVar->add_att("units", "m s-1");
            vVar->add_att("units", "m s-1");
        } else {
            file = new NcFile(fileName.c_str(), NcFile::Write);
            
            timeVar = file->get_var("time");
            uVar = file->get_var("u");
            vVar = file->get_var("v");
        }
        
        double seconds = TimeManager::getSeconds();
        int record = TimeManager::getSteps();
        timeVar->put_rec(&seconds, record);

        uVar->put_rec(u.data(), record);
        vVar->put_rec(v.data(), record);
        
        file->close();
        
        delete file;
    }

    int numLon, numLat;
    Array<double, 2> u, v;
    double *lon, *lat;
};

#endif