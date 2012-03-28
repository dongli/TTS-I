#ifndef PolygonRezoner_h
#define PolygonRezoner_h

class MeshManager;
class MeshAdaptor;
class FlowManager;
class PolygonManager;

namespace PolygonRezoner {
    void rezone(MeshManager &, const MeshAdaptor &,
                const FlowManager &, PolygonManager &);
}

#endif
