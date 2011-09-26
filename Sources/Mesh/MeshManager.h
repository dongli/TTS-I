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

    void init(int numLon, int numLat, double *lon, double *lat);
    void init(int numLon, int numLat, int numLev,
              double *lon, double *lat, double *lev);

    bool hasLayers() const;

    const RLLMesh &getMesh(RLLMesh::MeshType type) const { return mesh[type]; }
    const Layers &getLayers(Layers::LayerType type) const { return layers[type]; }

    void checkLocation(const Coordinate &x, Location &loc, Point *point = NULL);
    void countPoint(Point *point);

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
