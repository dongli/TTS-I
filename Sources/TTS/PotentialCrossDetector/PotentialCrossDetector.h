#ifndef PotentialCrossDetector_h
#define PotentialCrossDetector_h

class Vertex;
class Edge;
class EdgePointer;
class Polygon;
class MeshManager;
class FlowManager;
class PolygonManager;
#include "Vector.h"

namespace PotentialCrossDetector
{
    enum Status {
        NoCross, Cross
    };

    Status detectReplaceVertex(EdgePointer *edgePointer,
                               Vertex *oldVertex, Vertex *newVertex);

    Status detectRemoveVertexOnEdges(MeshManager &meshManager,
                                     EdgePointer *edgePointer,
                                     Vertex *testPoint, Polygon *polygon);

    Status detectInsertVertexOnEdge(MeshManager &meshManager,
                                    const FlowManager &flowManager,
                                    PolygonManager &polygonManager,
                                    Edge *oldEdge, Vertex *newVertex,
                                    Vertex *oldVertex = 0x0,
                                    Edge **crossedEdge = 0x0);

    Status detectAddConnection(Polygon *polygon,
                               EdgePointer *edgePointer1,
                               EdgePointer *&edgePointer2,
                               bool &isConnectOk1, bool &isConnectOk2,
                               Vector &vector1, Vector &vector2);

    Status detectTestPoint(EdgePointer *edgePointer1, EdgePointer *edgePointer2);

    void adjustMergeEdgeAngleThreshold(Edge *edge1, Edge *edge2, double &a0);
}

#endif
