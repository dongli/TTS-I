#include <netcdfcpp.h>

#include "TracerManager.h"

TracerManager::TracerManager()
{
#ifndef UNIT_TEST
    REPORT_ONLINE("TracerManager")
#endif
}

TracerManager::~TracerManager()
{
    tracers.destroy();
#ifndef UNIT_TEST
    REPORT_OFFLINE("TracerManager")
#endif
}

void TracerManager::init(const string &fileName)
{
    polygonManager.init(fileName);
}

void TracerManager::registerTracer(const string &fileName,
                                   const MeshManager &meshManager)
{
    // -------------------------------------------------------------------------
    NcFile *file = new NcFile(fileName.c_str(), NcFile::ReadOnly);

    if (!file->is_valid()) {
        ostringstream message;
        message << "Failed to open \"" << fileName << "\"!";
        REPORT_ERROR(message.str());
    }

    string name = file->get_att("name")->as_string(0);
    string long_name = file->get_att("long_name")->as_string(0);
    string unit = file->get_att("unit")->as_string(0);

    // -------------------------------------------------------------------------
    Tracer *tracer = tracers.front();
    for (int i = 0; i < tracers.size(); ++i) {
        if (tracer->getName() == name) {
            ostringstream message;
            message << "Tracer \"" << name << "\" has already been registered!";
            REPORT_ERROR(message.str())
        }
        tracer = tracer->next;
    }

    // -------------------------------------------------------------------------
    tracers.append(tracer);
    tracer->init(name, long_name, unit, meshManager, polygonManager);

    // -------------------------------------------------------------------------
    file->close();
    delete file;
}

void TracerManager::update()
{
    Tracer *tracer = tracers.front();
    for (int i = 0; i < tracers.size(); ++i) {
        tracer->update();
        tracer = tracer->next;
    }
}

void TracerManager::remap()
{
    // -------------------------------------------------------------------------
    // calculate the intersections between polygons and the mesh grid lines
    
}

void TracerManager::output(const string &fileName)
{
    polygonManager.output(fileName);
}