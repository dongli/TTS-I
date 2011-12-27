#ifndef CurvatureGuard_h
#define CurvatureGuard_h

class Edge;
class EdgePointer;
class Vertex;
class Polygon;
class MeshManager;
class FlowManager;
class PolygonManager;

#include "AngleThreshold.h"
#include <string>

using std::string;

namespace CurvatureGuard
{
    void init();

    void guard(MeshManager &, const FlowManager &, PolygonManager &);
    
    bool splitEdge(MeshManager &, const FlowManager &, PolygonManager &,
                   Edge *edge, bool isChecked = false, bool isMustSplit = false);
    bool splitEdges(MeshManager &, const FlowManager &, PolygonManager &);

    bool mergeEdges(MeshManager &, const FlowManager &, PolygonManager &);

    void splitPolygon(MeshManager &, const FlowManager &, PolygonManager &,
                      Polygon *&polygon1, EdgePointer *edgePointer1,
                      EdgePointer *edgePointer2, Vertex *vertex3, int mode);
    bool splitPolygons(MeshManager &, const FlowManager &, PolygonManager &);

    void calcAngleThreshold(Edge *edge, double &a0);
    void calcAngleThreshold(Edge *edge1, Edge *edge2, double &a0);
    void relaxAngleThreshold(Edge *edge1, Edge *edge2, double &a0);
}

#endif
