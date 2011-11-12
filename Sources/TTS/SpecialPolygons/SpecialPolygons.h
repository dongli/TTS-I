#ifndef SpecialPolygons_h
#define SpecialPolygons_h

class PolygonManager;
class Polygon;
class EdgePointer;

namespace SpecialPolygons
{
    void handleLinePolygon(PolygonManager &polygonManager,
                           Polygon *polygon);

    void handlePointPolygon(PolygonManager &polygonManager,
                            Polygon *polygon);

    void handleEnclosedPolygons(PolygonManager &polygonManager,
                                Polygon *polygon1,
                                EdgePointer *edgePointer11,
                                EdgePointer *edgePointer12,
                                Polygon *polygon = 0x0);
}

#endif
