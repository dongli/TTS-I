#ifndef SpecialPolygons_h
#define SpecialPolygons_h

class PolygonManager;
class Polygon;
class EdgePointer;
class MeshManager;
class FlowManager;

namespace SpecialPolygons
{
    void handleLinePolygon(PolygonManager &polygonManager, Polygon *&polygon);

    void handlePointPolygon(PolygonManager &polygonManager, Polygon *polygon);

    void handleSlimPolygon(MeshManager &meshManager,
                           const FlowManager &flowManager,
                           PolygonManager &polygonManager, Polygon *&polygon);
}

#endif
