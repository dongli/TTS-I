#ifndef PolygonRezoner_h
#define PolygonRezoner_h

class MeshManager;
class MeshAdaptor;
class FlowManager;
class TracerManager;

namespace PolygonRezoner {
    void init();

    void rezone(MeshManager &, MeshAdaptor &, const FlowManager &, TracerManager &);
}

#endif
