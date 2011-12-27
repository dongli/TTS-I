#ifndef ApproachDetector_h
#define ApproachDetector_h

class Vertex;
class Edge;
class EdgePointer;
class Polygon;
class PolygonManager;
class MeshManager;
class FlowManager;

#include "Projection.h"
#include "VertexAgent.h"
#include "EdgeAgent.h"
#include "AgentPair.h"
#include "ApproachingVertices.h"

namespace ApproachDetector
{
    double approachTrendThreshold(double distance);

    bool isNeedCheck(double distance);

    bool isApproaching(Projection *projection);

    bool checkApproachValid(MeshManager &meshManager,
                            const FlowManager &flowManager,
                            PolygonManager &polygonManager,
                            EdgePointer *edgePointer1,
                            EdgePointer *edgePointer2,
                            Vertex *vertex3);

    void detectPolygon(MeshManager &meshManager, const FlowManager &flowManager,
                       PolygonManager &polygonManager, Polygon *polygon);

    void detectPolygons(MeshManager &meshManager, const FlowManager &flowManager,
                        PolygonManager &polygonManager);

    void reset(PolygonManager &polygonManager);

    int chooseMode(EdgePointer *edgePointer1, Vertex *vertex3,
                   Projection *projection);
}

#endif
