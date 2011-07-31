#ifndef _MeshManager_h_
#define _MeshManager_h_

#include "Velocity.h"
#include "RLLMesh.h"
#include "Layers.h"
#include "Point.h"
#include "PointCounter.h"
#include "TimeManager.h"

class MeshManager
{
public:
    MeshManager();
    virtual ~MeshManager();

    void setPoleR(double PoleR);

    void construct(int numLon, int numLat, double *lon, double *lat);
    void construct(int numLon, int numLat, int numLev,
                   double *lon, double *lat, double *lev);

    bool hasLayers() const;

    const RLLMesh &getMesh(RLLMesh::MeshType type) const { return mesh[type]; }

    void checkLocation(const Coordinate &x, Location &loc, int ID = -1);

    void move(const Coordinate &x0, Coordinate &x1, const Velocity &v,
              Second dt, const Location &loc) const;

    void resetPointCounter() { pointCounter.reset(); }

private:
    friend class FlowManager;
    friend class TTS;

    double PoleR;
    RLLMesh mesh[4];
    Layers layers[2];
    PointCounter pointCounter;
};

#endif
