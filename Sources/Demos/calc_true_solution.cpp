#include "Constants.h"
#include "Sphere.h"
#include "TimeManager.h"
#include "MeshManager.h"
#include "MovingVortices.h"
#include <netcdfcpp.h>

#define MOVINGVORTICES_TESTCASE

int main(int argc, char **argv)
{
    MeshManager meshManager;
    Array<double, 2> q;
#ifdef MOVINGVORTICES_TESTCASE
    MovingVortices testCase;
    char fileName[30] = "mv_true_360x180.nc";
#endif
    
    // -------------------------------------------------------------------------
    TimeManager::setClock(1800.0);
    TimeManager::setEndStep(576);
    // -------------------------------------------------------------------------
    int numLon = 360, numLat = 180;
    double dlon = PI2/numLon;
    double dlat = PI/(numLat+1);
    double *lon = new double[numLon];
    double *lat = new double[numLat];
    for (int i = 0; i < numLon; ++i)
        lon[i] = i*dlon;
    for (int j = 0; j < numLat; ++j)
        lat[j] = PI05-(j+1)*dlat;
    meshManager.init(numLon, numLat, lon, lat);
    const RLLMesh meshCnt = meshManager.getMesh(PointCounter::Center);
    q.resize(meshCnt.lat.size(), meshCnt.lon.size());
    // -------------------------------------------------------------------------
    NcFile file(fileName, NcFile::Replace);
    NcDim *timeDim = file.add_dim("time");
    NcDim *latDim = file.add_dim("lat", q.extent(0));
    NcDim *lonDim = file.add_dim("lon", q.extent(1));
    NcVar *timeVar = file.add_var("time", ncDouble, timeDim);
    NcVar *latVar = file.add_var("lat", ncDouble, latDim);
    NcVar *lonVar = file.add_var("lon", ncDouble, lonDim);
    NcVar *qVar = file.add_var("q", ncDouble, timeDim, latDim, lonDim);
    delete [] lon;
    delete [] lat;
    lat = new double[q.extent(0)];
    lon = new double[q.extent(1)];
    for (int i = 0; i < q.extent(0); ++i)
        lat[i] = meshCnt.lat(i)*Rad2Deg;
    for (int i = 0; i < q.extent(1); ++i)
        lon[i] = meshCnt.lon(i)*Rad2Deg;
    latVar->put(lat, q.extent(0));
    lonVar->put(lon, q.extent(1));
    // -------------------------------------------------------------------------
    double time = 0.0;
    testCase.calcSolution(time, meshCnt.lon, meshCnt.lat, q);
    timeVar->put_rec(&time, TimeManager::getSteps());
    qVar->set_rec(TimeManager::getSteps());
    qVar->put_rec(q.data());

    // -------------------------------------------------------------------------
    while (!TimeManager::isFinished()) {
        TimeManager::advance();
        time = TimeManager::getSeconds();
        testCase.calcSolution(time, meshCnt.lon, meshCnt.lat, q);
        timeVar->put_rec(&time, TimeManager::getSteps());
        qVar->set_rec(TimeManager::getSteps());
        qVar->put_rec(q.data());
    }
    file.close();
}