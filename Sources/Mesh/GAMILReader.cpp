#include "GAMILReader.h"
#include "SystemCalls.h"
#include "TimeManager.h"
#include <netcdfcpp.h>
#include <sys/stat.h>
#include <blitz/array.h>

using blitz::Array;
using blitz::Range;

GAMILReader::GAMILReader()
{
    REPORT_ONLINE("GAMILReader")
}

GAMILReader::~GAMILReader()
{
    REPORT_OFFLINE("GAMILReader")
}

void GAMILReader::init(const string &dir, const string &filePattern)
{
    // -------------------------------------------------------------------------
    // get all the file names
    SystemCalls::getFiles(dir, filePattern, fileNames);
    // -------------------------------------------------------------------------
    NcFile file(fileNames[0].c_str(), NcFile::ReadOnly);
    // get the mesh information
    int numLon = static_cast<int>(file.get_dim("lon_full")->size());
    int numLat = static_cast<int>(file.get_dim("lat_full")->size());
    Array<double, 1> lon(numLon), lat(numLat);
    file.get_var("lon")->get(lon.data(), numLon);
    file.get_var("lat")->get(lat.data(), numLat);
    lon /= Rad2Deg;
    lat /= Rad2Deg;
    // get the time information
    double time;
    file.get_var("time")->get(&time);
    TimeManager::setClock(1200, time*86400.0);
    TimeManager::setEndStep(static_cast<int>(fileNames.size())-1);
    file.close();
    // -------------------------------------------------------------------------
    meshManager.init(numLon, numLat, lon.data(), lat.data());
    meshAdaptor.init(meshManager);
    flowManager.init(meshManager);
}

void GAMILReader::getTracerField(TracerManager &tracerManager)
{
    // -------------------------------------------------------------------------
    // moisture field
    const RLLMesh &meshCnt = meshManager.getMesh(PointCounter::Center);
    const RLLMesh &meshBnd = meshManager.getMesh(PointCounter::Bound);
    Field q0; q0.init(meshCnt, meshBnd);
    // -------------------------------------------------------------------------
    // read in moisture
    if (false) {
        NcFile file("gamil_data/q.nc", NcFile::ReadOnly);
        if (!file.is_valid()) {
            ostringstream message;
            message << "Failed to open file q.nc";
            REPORT_ERROR(message.str())
        }
        int numLon = static_cast<int>(file.get_dim("lon")->size());
        int numLat = static_cast<int>(file.get_dim("lat")->size());
        int numLev = static_cast<int>(file.get_dim("lev")->size());
        Array<double, 3> q(numLat, numLev, numLon);
        file.get_var("q")->get(q.data(), 1, numLat, numLev, numLon);
        file.close();
        for (int i = 0; i < meshCnt.getNumLon(); ++i)
            for (int j = 0; j < meshCnt.getNumLat(); ++j)
                q0.values(i, j) = q(j, 12, i);
    } else {
        // ideal unity field
        for (int i = 0; i < meshCnt.getNumLon(); ++i)
            for (int j = 0; j < meshCnt.getNumLat(); ++j)
                q0.values(i, j) = 1.0;
    }
    // -------------------------------------------------------------------------
    tracerManager.registerTracer("test tracer 0", "", meshManager);
    // check the location polygon vertices
    Vertex *vertex = tracerManager.polygonManager.vertices.front();
    for (int i = 0; i < tracerManager.polygonManager.vertices.size(); ++i) {
        Location loc;
        meshManager.checkLocation(vertex->getCoordinate(), loc, vertex);
        vertex->setLocation(loc);
        vertex = vertex->next;
    }
    // -------------------------------------------------------------------------
    // remap the air density onto the polygons
    meshAdaptor.adapt(tracerManager, meshManager);
    meshAdaptor.remap("test tracer 0", q0, tracerManager);
    meshAdaptor.remap("test tracer 0", tracerManager);
}

void GAMILReader::getVelocityField()
{
    cout << "reading " << fileNames[TimeManager::getSteps()] << endl;

    NcError ncError(NcError::silent_nonfatal);

    NcFile file(fileNames[TimeManager::getSteps()].c_str(), NcFile::ReadOnly);

    if (!file.is_valid()) {
        ostringstream message;
        message << "Failed to open file " << fileNames[TimeManager::getSteps()];
        REPORT_ERROR(message.str())
    }

    int numLon = static_cast<int>(file.get_dim("lon_full")->size());
    int numLat = static_cast<int>(file.get_dim("lat_full")->size());
    int numLonHalf = static_cast<int>(file.get_dim("lon_half")->size());
    int numLatHalf = static_cast<int>(file.get_dim("lat_half")->size());
    int numLev = static_cast<int>(file.get_dim("lev")->size());
    Array<double, 3> a(numLev, numLat, numLonHalf), b(numLev, numLatHalf, numLon);
    file.get_var("us")->get(a.data(), 1, numLev, numLat, numLonHalf);
    file.get_var("vs")->get(b.data(), 1, numLev, numLatHalf, numLon);
    file.close();

    double u[numLonHalf][numLat], v[numLon][numLatHalf];
    for (int i = 0; i < numLonHalf; ++i)
        for (int j = 0; j < numLat; ++j)
                u[i][j] = a(12, j, i);
    for (int i = 0; i < numLon; ++i)
        for (int j = 0; j < numLatHalf; ++j)
                v[i][j] = b(12, j, i);

    flowManager.update(&u[0][0], &v[0][0]);
}

#ifdef DEBUG
void GAMILReader::checkVelocityField()
{
    int numLon = 360, numLat = 181;
    double lon[numLon], lat[numLat];
    double dlon = PI2/numLon, dlat = PI/(numLat-1);

    for (int i = 0; i < numLon; ++i)
        lon[i] = i*dlon;
    for (int j = 0; j < numLat; ++j)
        lat[j] = PI05-j*dlat;

    double u[numLat][numLon], v[numLat][numLon];

    for (int j = 0; j < numLat; ++j)
        for (int i = 0; i < numLon; ++i) {
            Coordinate x;
            x.setSPH(lon[i], lat[j]);
            Location loc;
            meshManager.checkLocation(x, loc);
            Velocity velocity;
            flowManager.getVelocity(x, loc, NewTimeLevel,
                                    velocity, Velocity::LonLatSpace);
            u[j][i] = velocity.u;
            v[j][i] = velocity.v;
        }
    for (int i = 0; i < numLon; ++i)
        lon[i] *= Rad2Deg;
    for (int j = 0; j < numLat; ++j)
        lat[j] *= Rad2Deg;

    string fileName = "debug_gamil.suv.nc";
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

        lonVar->put(lon, numLon);
        latVar->put(lat, numLat);

        uVar = file->add_var("u", ncDouble, timeDim, latDim, lonDim);
        vVar = file->add_var("v", ncDouble, timeDim, latDim, lonDim);
    } else {
        timeVar = file->get_var("time");
        uVar = file->get_var("u");
        vVar = file->get_var("v");
    }

    double seconds = TimeManager::getSeconds();
    int record = TimeManager::getSteps();
    timeVar->put_rec(&seconds, record);

    uVar->put_rec(&u[0][0], record);
    vVar->put_rec(&v[0][0], record);

    file->close();

    delete file;
}
#endif
