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
    //! \brief Calculate the threshold for the approaching events.
    //! \param distance Distance between the vertex and the edge.
    //! \return The minimal distance change ratio that an approaching
    //!         event should satisfy.
    double approachTrendThreshold(double distance);

    //! \brief Judge whether the vertex and edge should be checked.
    //! \param distance The distance between vertex and edge.
    //! \return The boolean status.
    bool isNeedCheck(double distance);

    //! \brief Judge whether the approaching event occurs for the
    //!        given projection.
    //! \param projection Projection on the edge of the vertex.
    //! \return The boolean status of approaching.
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
