#ifndef SpecialPolygons_h
#define SpecialPolygons_h

class PolygonManager;
class Polygon;
class EdgePointer;
class MeshManager;
class FlowManager;

namespace SpecialPolygons
{
    void handleLinePolygon(PolygonManager &polygonManager, Polygon *polygon,
                           bool isKeepMass = false);

    void handlePointPolygon(PolygonManager &polygonManager, Polygon *polygon,
                            bool isKeepMass = false);

    bool handleSlimPolygon(MeshManager &meshManager,
                           const FlowManager &flowManager,
                           PolygonManager &polygonManager, Polygon *polygon,
                           bool isKeepMass = false);
}

#endif
