#ifndef PolygonRezoner_h
#define PolygonRezoner_h

class MeshManager;
class FlowManager;
class PolygonManager;

namespace PolygonRezoner {
    void rezone(MeshManager &, const FlowManager &, PolygonManager &);
}

#endif
