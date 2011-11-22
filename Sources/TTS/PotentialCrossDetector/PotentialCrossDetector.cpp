#include "PotentialCrossDetector.h"
#include "ApproachDetector.h"
#include "Vertex.h"
#include "Edge.h"

using namespace PotentialCrossDetector;
using namespace ApproachDetector;

//! Detect the potential edge-crossing event (CASE 1).
/*!
 When the approaching vertex (vertex3) will be replaced by the other vertex
 (testVertex, may be vertex1, vertex2 or a new vertex), there is a possibility
 that this operation will cause the approaching vertices of vertex3's linked
 edges crossing the edges. So when this happens, skip vertex3, but dead loop
 may also occur, that is, the two vertices cause each other crossing its paired
 edge, for the time being, we just ignore them.
 */

Status PotentialCrossDetector::detect1(Vertex *vertex3, Vertex *testVertex)
{
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
            OrientStatus orient;
            orient = Sphere::orient(vertex1, vertex2, *it);
            if (orient != projection->getOrient() && orient != OrientOn) {
                if (projection->isApproaching())
                    ApproachingVertices::jumpVertex(vertex3, *it);
                return Cross;
            }
        }
        linkedEdge = linkedEdge->next;
    }
    return NoCross;
}

//*! Detect the potential edge-crossing event (CASE 2).
/*!
 In the neighbor polygon (polygon2), check the edge angle on each end, if it is
 smaller than a threshold, check if the test vertex will cross one edge.
 */

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
    edgePointer2 = NULL;
    if (edgePointer1->next->getAngle(NewTimeLevel) < smallAngle) {
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

//! Detect the potential edge-crossing event (CASE 3).
/*!
 When merge two edges, the vertices that have projections on these two edges may
 cross the new edge. If so these two edges will not be merged.
 */

Status PotentialCrossDetector::detect3(Edge *edge1, Edge *edge2)
{
    int mode; Vertex *vertex1, *vertex2, *vertex3;
    if (edge1->getEndPoint(SecondPoint) == edge2->getEndPoint(FirstPoint)) {
        mode = 1;
        vertex1 = edge1->getEndPoint(FirstPoint);
        vertex2 = edge1->getEndPoint(SecondPoint);
        vertex3 = edge2->getEndPoint(SecondPoint);
    } else if (edge1->getEndPoint(FirstPoint) == edge2->getEndPoint(FirstPoint)) {
        mode = 2;
        vertex1 = edge1->getEndPoint(SecondPoint);
        vertex2 = edge1->getEndPoint(FirstPoint);
        vertex3 = edge2->getEndPoint(SecondPoint);
    } else if (edge1->getEndPoint(SecondPoint) == edge2->getEndPoint(SecondPoint)) {
        mode = 3;
        vertex1 = edge1->getEndPoint(FirstPoint);
        vertex2 = edge1->getEndPoint(SecondPoint);
        vertex3 = edge2->getEndPoint(FirstPoint);
    } else if (edge1->getEndPoint(FirstPoint) == edge2->getEndPoint(SecondPoint)) {
        mode = 4;
        vertex1 = edge1->getEndPoint(SecondPoint);
        vertex2 = edge1->getEndPoint(FirstPoint);
        vertex3 = edge2->getEndPoint(FirstPoint);
    } else {
        Message message;
        message << "Edge " << edge1->getID() << " and " << edge2->getID();
        message << " do not share a common end point!";
        REPORT_ERROR(message.str());
    }
    OrientStatus orient = Sphere::orient(vertex1, vertex3, vertex2);
    if (orient == OrientOn) return NoCross;
    std::list<Vertex *>::const_iterator it;
    Projection *projection;
    // edge1
    for (it = edge1->detectAgent.vertices.begin();
         it != edge1->detectAgent.vertices.end(); ++it) {
        projection = (*it)->detectAgent.getProjection(edge1);
        if (mode == 1 || mode == 3) {
            if (orient == OrientLeft) {
                if (projection->getOrient() == OrientRight)
                    if (Sphere::orient(vertex1, vertex3, *it) == OrientLeft)
                        return Cross;
            } else {
                if (projection->getOrient() == OrientLeft)
                    if (Sphere::orient(vertex1, vertex3, *it) == OrientRight)
                        return Cross;
            }
        } else
            if (projection->getOrient() == Sphere::orient(vertex1, vertex3, *it))
                return Cross;
    }
    // edge2
    for (it = edge2->detectAgent.vertices.begin();
         it != edge2->detectAgent.vertices.end(); ++it) {
        projection = (*it)->detectAgent.getProjection(edge2);
        if (mode == 1 || mode == 2) {
            if (orient == OrientLeft) {
                if (projection->getOrient() == OrientRight)
                    if (Sphere::orient(vertex1, vertex3, *it) == OrientLeft)
                        return Cross;
            } else {
                if (projection->getOrient() == OrientLeft)
                    if (Sphere::orient(vertex1, vertex3, *it) == OrientRight)
                        return Cross;
            }
        } else
            if (projection->getOrient() == Sphere::orient(vertex1, vertex3, *it))
                return Cross;
    }
    return NoCross;
}