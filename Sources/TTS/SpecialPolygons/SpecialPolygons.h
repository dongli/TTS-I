#ifndef SpecialPolygons_h
#define SpecialPolygons_h

class PolygonManager;
class Polygon;
class Vertex;
class EdgePointer;
class MeshManager;
class FlowManager;

namespace SpecialPolygons
{
    void handleLinePolygon(PolygonManager &polygonManager, Polygon *polygon,
                           Vertex *keepVertex = 0x0, bool isKeepMass = false);

    void handlePointPolygon(PolygonManager &polygonManager, Polygon *polygon,
                            bool isKeepMass = false);

    bool handleSlimPolygon(MeshManager &meshManager,
                           const FlowManager &flowManager,
                           PolygonManager &polygonManager, Polygon *polygon,
                           bool isKeepMass = false);

    void handleEnclosedPolygons(EdgePointer *edgePointer1,
                                EdgePointer *edgePointer2,
                                PolygonManager &polygonManager);
}

#endif
