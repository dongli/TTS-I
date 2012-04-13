#ifndef TracerManager_h
#define TracerManager_h

#include "MeshManager.h"
#include "PolygonManager.h"
#include "Field.h"

class TracerManager
{
public:
	TracerManager();
	virtual ~TracerManager();

    void init();

    void registerTracer(const string &tracerName, const string &units,
                        const MeshManager &meshManager);
    void registerTracer(const string &fileName,
                        const MeshManager &meshManager);

    int getTracerNum() const { return static_cast<int>(tracerNames.size()); }
    const string &getTracerName(int tracerId) const { return tracerNames[tracerId]; }
    int getTracerId(const string &tracerName);
    Field &getTracerDensityField(int tracerId) { return tracerDensities[tracerId]; }

    void update();

    void output(const string &fileName);

    PolygonManager polygonManager;

private:
    vector<string> tracerNames, tracerUnits;
    vector<Field> tracerDensities;
};

#endif
