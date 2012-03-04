#include <netcdfcpp.h>

#include "TracerManager.h"
#include "Tracer.h"

TracerManager::TracerManager()
{
    REPORT_ONLINE("TracerManager");
}

TracerManager::~TracerManager()
{
    REPORT_OFFLINE("TracerManager");
}

void TracerManager::init(const string &fileName)
{
    polygonManager.init(fileName);
}

void TracerManager::registerTracer(const string &tracerName,
                                   const string &tracerUnit,
                                   const MeshManager &meshManager)
{
    NOTICE("TracerManager", "Register tracer \""+tracerName+"\"");
    tracerNames.push_back(tracerName);
    tracerUnits.push_back(tracerUnit);

    // -------------------------------------------------------------------------
    // initialize tracer density on the mesh
    tracerDensities.push_back(Field());
    if (meshManager.hasLayers())
        tracerDensities.back().init(meshManager.getMesh(PointCounter::Center),
                                    meshManager.getMesh(PointCounter::Bound),
                                    meshManager.getLayers(Layers::Full));
    else
        tracerDensities.back().init(meshManager.getMesh(PointCounter::Center),
                                    meshManager.getMesh(PointCounter::Bound));

    // -------------------------------------------------------------------------
    // set up tracer variables in polygons
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        polygon->tracers.push_back(Tracer());
        polygon = polygon->next;
    }
}

void TracerManager::registerTracer(const string &fileName,
                                   const MeshManager &meshManager)
{
    // -------------------------------------------------------------------------
    // parse and read file
    NcError ncError(NcError::silent_nonfatal);

    NcFile file(fileName.c_str(), NcFile::ReadOnly);
    if (!file.is_valid()) {
        Message message;
        message << "Failed to open tracer file \"" << fileName << "\"!";
        REPORT_ERROR(message.str());
    }

    if (file.get_att("name") == NULL) {
        Message message;
        message << "There is no \"name\" attribute in tracer file \"";
        message << fileName << "\"!";
        REPORT_ERROR(message.str());
    }
    tracerNames.push_back(file.get_att("name")->as_string(0));

    if (file.get_att("name") == NULL) {
        Message message;
        message << "There is no \"name\" attribute in tracer file \"";
        message << fileName << "\"!";
        REPORT_ERROR(message.str());
    }
    tracerNames.push_back(file.get_att("name")->as_string(0));

    int numPolygon = static_cast<int>(file.get_dim("num_total_polygon")->size());
    if (numPolygon != polygonManager.polygons.size()) {
        Message message;
        message << "Polygon numbers (" << numPolygon << " != ";
        message << polygonManager.polygons.size();
        message << ") are not consistent in tracer file \"";
        message << fileName << "\"!";
        REPORT_ERROR(message.str());
    }
    double mass[numPolygon];
    if (file.get_var("mass") == NULL) {
        Message message;
        message << "There is no \"mass\" variable in tracer file \"";
        message << fileName << "\"!";
        REPORT_ERROR(message.str());
    }
    file.get_var("mass")->get(mass);
    // -------------------------------------------------------------------------
    // initialize tracer density on the mesh
    tracerDensities.push_back(Field());
    if (meshManager.hasLayers())
        tracerDensities.back().init(meshManager.getMesh(PointCounter::Center),
                                    meshManager.getMesh(PointCounter::Bound),
                                    meshManager.getLayers(Layers::Full));
    else
        tracerDensities.back().init(meshManager.getMesh(PointCounter::Center),
                                    meshManager.getMesh(PointCounter::Bound));
    // -------------------------------------------------------------------------
    // set up tracer variables in polygons
    int tracerId = static_cast<int>(tracerNames.size()-1);
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        polygon->tracers.push_back(Tracer());
        polygon->tracers[tracerId].setMass(mass[i]);
        polygon = polygon->next;
    }
}

int TracerManager::getTracerId(const string &tracerName)
{
    for (int i = 0; i < tracerNames.size(); ++i)
        if (tracerNames[i] == tracerName)
            return i;
    Message message;
    message << "There is no tracer \"" << tracerName << "\"!";
    REPORT_ERROR(message.str());
}

void TracerManager::update()
{
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        polygon->updateTracers();
        polygon = polygon->next;
    }
}

void TracerManager::output(const string &fileName)
{
    // -------------------------------------------------------------------------
    // output polygon stuffs
    polygonManager.output(fileName);
    // -------------------------------------------------------------------------
    NcFile file(fileName.c_str(), NcFile::Write);
    if (!file.is_valid()) {
        Message message;
        message << "Failed to open tracer output file \"";
        message << fileName << "\" for appending meshed density field!";
        REPORT_ERROR(message.str());
    }
    // -------------------------------------------------------------------------
    // output tracer densities on the polygons
    NcDim *numPolygonDim = file.get_dim("num_total_polygon");
    double q0[polygonManager.polygons.size()];
    for (int l = 0; l < tracerNames.size(); ++l) {
        char varName[30];
        sprintf(varName, "q%d", l);
        NcVar *qVar = file.add_var(varName, ncDouble, numPolygonDim);
        Polygon *polygon = polygonManager.polygons.front();
        for (int i = 0; i < polygonManager.polygons.size(); ++i) {
            q0[i] = polygon->tracers[l].getDensity();
            polygon = polygon->next;
        }
        qVar->put(q0, polygonManager.polygons.size());
    }
#ifdef TTS_REMAP
    // -------------------------------------------------------------------------
    // output tracer densities on the mesh
    int numLon = tracerDensities[0].values.extent(0);
    int numLat = tracerDensities[0].values.extent(1);
    double lon[numLon], lat[numLat];
    for (int i = 0; i < numLon; ++i)
        lon[i] = tracerDensities[0].getMesh().lon(i)*Rad2Deg;
    for (int j = 0; j < numLat; ++j)
        lat[j] = tracerDensities[0].getMesh().lat(j)*Rad2Deg;
    NcDim *lonDim = file.add_dim("lon", numLon);
    NcDim *latDim = file.add_dim("lat", numLat);
    NcVar *lonVar = file.add_var("lon", ncDouble, lonDim);
    lonVar->add_att("long_name", "longitude");
    lonVar->add_att("units", "degrees_east");
    lonVar->put(lon, numLon);
    NcVar *latVar = file.add_var("lat", ncDouble, latDim);
    latVar->add_att("long_name", "latitude");
    latVar->add_att("units", "degrees_north");
    latVar->put(lat, numLat);
    NcVar *areaVar = file.add_var("area_mesh", ncDouble, latDim, lonDim);
    areaVar->add_att("long_name", "area of fixed mesh cell");
    areaVar->add_att("units", "m2");
    double area[numLat][numLon];
    for (int i = 0; i < numLon; ++i)
        for (int j = 0; j < numLat; ++j)
            area[j][i] = tracerDensities[0].getMesh(Field::Bound).area(i, j);
    areaVar->put(&area[0][0], numLat, numLon);
    double q[numLat][numLon];
    for (int l = 0; l < tracerNames.size(); ++l) {
        char varName[30];
        sprintf(varName, "q%d_mesh", l);
        NcVar *qVar = file.add_var(varName, ncDouble, latDim, lonDim);
        qVar->add_att("long_name", tracerNames[l].c_str());
        for (int i = 0; i < numLon; ++i)
            for (int j = 0; j < numLat; ++j)
                q[j][i] = tracerDensities[l].values(i, j, 0).getNew();
        qVar->put(&q[0][0], numLat, numLon);
    }
#endif
    // -------------------------------------------------------------------------
    file.close();
    NOTICE("TracerManager", fileName+" is generated.");
}
