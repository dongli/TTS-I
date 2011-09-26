#ifndef _ParcelManager_h_
#define _ParcelManager_h_

#include <vector>

#include "PolygonManager.h"
#include "Tracer.h"

class TracerManager
{
public:
	TracerManager();
	virtual ~TracerManager();

    void init(const string &fileName);

    void registerTracer(const string &fileName, const MeshManager &);

    void update();
    void remap();

    void output(const string &fileName);

    PolygonManager polygonManager;
    List<Tracer> tracers;
};

#endif
