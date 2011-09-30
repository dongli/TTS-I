#include "PotentialCrossDetector.h"
#include "ApproachDetector.h"
#include "Vertex.h"
#include "Edge.h"

using namespace PotentialCrossDetector;
using namespace ApproachDetector;

DeadLoopPair PotentialCrossDetector::deadLoopPair;

bool PotentialCrossDetector::isDeadLoop()
{
    return deadLoopPair.vertex1 == NULL ? false : true;
}

//! Detect the potential edge-crossing event (CASE 1).
/*!
 When the approaching vertex (vertex3) will be replaced by the other vertex
 (testVertex, may be vertex1, vertex2 or a new vertex), there is a possibility
 that this operation will cause the approaching vertices of vertex3's linked
 edges crossing the edges. So when this happens, skip vertex3, but dead loop
 may also occur, that is, the two vertices cause each other crossing its paired
 edge, for the time being, we just ignore them.
 */

Status PotentialCrossDetector::detect1(Vertex *vertex3, Vertex *testVertex,
                                       Edge *edge1)
{
    static Vertex *prevVertex1 = NULL, *prevVertex2 = NULL;
    EdgePointer *linkedEdge = vertex3->linkedEdges.front();
    for (int i = 0; i < vertex3->linkedEdges.size(); ++i) {
        Edge *edge = linkedEdge->edge;
        Vertex *vertex1 = NULL, *vertex2 = NULL;
        if (edge->getEndPoint(FirstPoint) == vertex3) {
            vertex1 = testVertex;
            vertex2 = edge->getEndPoint(SecondPoint);
        } else if (edge->getEndPoint(SecondPoint) == vertex3) {
            vertex1 = edge->getEndPoint(FirstPoint);
            vertex2 = testVertex;
        }
        std::list<Vertex *>::const_iterator it;
        for (it = edge->detectAgent.vertices.begin();
             it != edge->detectAgent.vertices.end(); ++it) {
            Projection *projection = (*it)->detectAgent.getProjection(edge);
            // Note: Here we check all the approaching vertices.
            //if ((projection != NULL && projection->isApproaching()) ||
            //    (*it == edge1->getEndPoint(FirstPoint) ||
            //     *it == edge1->getEndPoint(SecondPoint))) {
                OrientStatus orient;
                orient = Sphere::orient(vertex1->getCoordinate(),
                                        vertex2->getCoordinate(),
                                        (*it)->getCoordinate());
                if (orient != projection->getOrient() && orient != OrientOn) {
                    if (prevVertex1 == *it && prevVertex2 == vertex3) {
                        deadLoopPair.vertex1 = vertex3;
                        deadLoopPair.vertex2 = *it;
                        deadLoopPair.edge1 = edge1;
                        deadLoopPair.edge2 = edge;
                        REPORT_WARNING("Dead loop has occurred!")
                        return DeadLoop;
                    }
                    prevVertex1 = vertex3;
                    prevVertex2 = *it;
                    if (projection->isApproaching())
                        ApproachingVertices::jumpVertex(vertex3, *it);
                    return Cross;
                }
            //}
        }
        linkedEdge = linkedEdge->next;
    }
    // reset deadLoopPair
    deadLoopPair.vertex1 = NULL;
    return NoCross;
}

Status PotentialCrossDetector::detect2(Vertex *testVertex, Edge *edge1,
                                       Polygon *polygon2)
{
    static const double smallAngle = 20.0/Rad2Deg;
    Vertex *vertex1, *vertex2;
    EdgePointer *edgePointer1;
    if (edge1->getPolygon(OrientLeft) == polygon2) {
        edgePointer1 = edge1->getEdgePointer(OrientLeft);
    } else if (edge1->getPolygon(OrientRight) == polygon2) {
        edgePointer1 = edge1->getEdgePointer(OrientRight);
    }
    EdgePointer *edgePointer2 = NULL;
    if (edgePointer1->getAngle(NewTimeLevel) < smallAngle) {
        edgePointer2 = edgePointer1->prev;
    } else if (edgePointer1->next->getAngle(NewTimeLevel) < smallAngle) {
        edgePointer2 = edgePointer1->next;
    }
    if (edgePointer2 != NULL) {
        vertex1 = edgePointer2->getEndPoint(FirstPoint);
        vertex2 = edgePointer2->getEndPoint(SecondPoint);
        OrientStatus orient = Sphere::orient(vertex1->getCoordinate(),
                                             vertex2->getCoordinate(),
                                             testVertex->getCoordinate());
        if (orient == OrientRight)
            return Cross;
    }
    return NoCross;
}