#include "PotentialCrossDetector.h"
#include "ApproachDetector.h"
#include "CurvatureGuard.h"
#include "Vertex.h"
#include "Edge.h"
#include "MeshManager.h"

using namespace PotentialCrossDetector;
using namespace ApproachDetector;
using namespace CurvatureGuard;

Status PotentialCrossDetector::detectReplaceVertex(EdgePointer *edgePointer,
                                                   Vertex *oldVertex,
                                                   Vertex *newVertex)
{
    static std::list<Vertex *>::const_iterator itVtx;
    static std::list<Projection>::const_iterator itPrj;
    Vertex *vertex1, *vertex2, *vertex3, *vertex4;
    Projection *projection;
    OrientStatus orient;
    Edge *edge;
    EdgePointer *linkedEdge = oldVertex->linkedEdges.front();
    for (int i = 0; i < oldVertex->linkedEdges.size(); ++i) {
        // ---------------------------------------------------------------------
        // check the paired vertices of the linked edge
        edge = linkedEdge->edge;
        vertex1 = NULL, vertex2 = NULL;
        if (edge->getEndPoint(FirstPoint) == oldVertex) {
            vertex1 = newVertex;
            vertex2 = edge->getEndPoint(SecondPoint);
        } else if (edge->getEndPoint(SecondPoint) == oldVertex) {
            vertex1 = edge->getEndPoint(FirstPoint);
            vertex2 = newVertex;
        }
        if (vertex1 != vertex2) {
            for (itVtx = edge->detectAgent.vertices.begin();
                 itVtx != edge->detectAgent.vertices.end(); ++itVtx) {
                vertex3 = *itVtx;
                if (vertex3->getID() == -1 ||
                    vertex3 == vertex1 || vertex3 == vertex2)
                    continue;
                projection = vertex3->detectAgent.getProjection(edge);
                orient = Sphere::orient(vertex1, vertex2, vertex3);
                if (orient != projection->getOrient()) {
                    if (projection->tags.isSet(Approaching))
                        ApproachingVertices::jumpVertex(oldVertex, vertex3);
                    return Cross;
                }
            }
        } else {
            linkedEdge = linkedEdge->next;
            continue;
        }
        // ---------------------------------------------------------------------
        // check other linked edges
        if (edge->getEndPoint(FirstPoint) == oldVertex)
            vertex1 = edge->getEndPoint(SecondPoint);
        else
            vertex1 = edge->getEndPoint(FirstPoint);
        EdgePointer *otherLinkedEdge = oldVertex->linkedEdges.front();
        for (int j = 0; j < oldVertex->linkedEdges.size(); ++j) {
            if (otherLinkedEdge != linkedEdge) {
                edge = otherLinkedEdge->edge;
                // check linked edges of paired vertices
                for (itVtx = edge->detectAgent.vertices.begin();
                     itVtx != edge->detectAgent.vertices.end(); ++itVtx) {
                    if ((*itVtx)->getID() == -1) continue;
                    EdgePointer *vertexLinkedEdge = (*itVtx)->linkedEdges.front();
                    for (int k = 0; k < (*itVtx)->linkedEdges.size(); ++k) {
                        vertex2 = vertexLinkedEdge->edge->getEndPoint(FirstPoint);
                        vertex3 = vertexLinkedEdge->edge->getEndPoint(SecondPoint);
                        if ((vertex2 == oldVertex || vertex3 == oldVertex ||
                             vertex2 == newVertex || vertex3 == newVertex) ||
                            vertexLinkedEdge->edge == edgePointer->edge) {
                            vertexLinkedEdge = vertexLinkedEdge->next;
                            continue;
                        }
                        if (vertex2 == vertex1 || vertex3 == vertex1) {
                            OrientStatus orient1, orient2, orient3;
                            orient1 = Sphere::orient(oldVertex, vertex1, *itVtx);
                            orient2 = Sphere::orient(vertex1, newVertex, *itVtx);
                            orient3 = Sphere::orient(newVertex, oldVertex, *itVtx);
                            if (orient1 == orient2 && orient1 == orient3 &&
                                orient1 != OrientOn)
                                return Cross;
                        } else {
                            if (Sphere::isIntersect(vertex1, newVertex,
                                                    vertex2, vertex3))
                                return Cross;
                        }
                        vertexLinkedEdge = vertexLinkedEdge->next;
                    }
                }
                // check the edge itself
                if (edge->getEndPoint(FirstPoint) == oldVertex)
                    vertex2 = edge->getEndPoint(SecondPoint);
                else
                    vertex2 = edge->getEndPoint(FirstPoint);
                EdgePointer *vertexLinkedEdge = vertex2->linkedEdges.front();
                for (int k = 0; k < vertex2->linkedEdges.size(); ++k) {
                    if (vertexLinkedEdge->edge != edge &&
                        vertexLinkedEdge->edge != edgePointer->edge) {
                        vertex3 = vertexLinkedEdge->edge->getEndPoint(FirstPoint);
                        vertex4 = vertexLinkedEdge->edge->getEndPoint(SecondPoint);
                        if (vertex3 != newVertex && vertex4 != newVertex &&
                            vertex3 != vertex1 && vertex4 != vertex1 &&
                            Sphere::isIntersect(vertex1, newVertex, vertex3, vertex4))
                            return Cross;
                    }
                    vertexLinkedEdge = vertexLinkedEdge->next;
                }
            }
            otherLinkedEdge = otherLinkedEdge->next;
        }
        // ---------------------------------------------------------------------
        // check the paired edges of the old vertex
        for (itPrj = oldVertex->detectAgent.getProjections().begin();
             itPrj != oldVertex->detectAgent.getProjections().end(); ++itPrj) {
            edge = (*itPrj).getEdge();
            if (edge == edgePointer->edge)
                continue;
            if (vertex1 == edge->getEndPoint(FirstPoint)) {
                vertex2 = edge->getEndPoint(SecondPoint);
            } else if (vertex1 == edge->getEndPoint(SecondPoint)) {
                vertex2 = edge->getEndPoint(FirstPoint);
            } else
                continue;
            if (vertex2 == newVertex ||
                !Sphere::isProject(vertex1, newVertex, vertex2))
                continue;
            if (Sphere::orient(vertex1, vertex2, oldVertex) !=
                Sphere::orient(vertex1, vertex2, newVertex))
                return Cross;
        }
        linkedEdge = linkedEdge->next;
    }
    return NoCross;
}

Status PotentialCrossDetector::detectInsertVertexOnEdge
(MeshManager &meshManager, const FlowManager &flowManager,
 PolygonManager &polygonManager, Edge *oldEdge, Vertex *newVertex,
 Vertex *oldVertex, Edge **crossedEdge)
{
    // -------------------------------------------------------------------------
    Vertex *vertex1, *vertex2, *vertex3, *vertex4, *vertices[3];
    OrientStatus orient;
    static std::list<Projection>::iterator itPrj;
    static std::list<Vertex *>::iterator itVtx;
    Projection *projection;
    EdgePointer *linkedEdge;
    Edge *edge3;
    static std::list<Edge *> checkedEdges;
    checkedEdges.clear();
    // -------------------------------------------------------------------------
    // collect information
    vertex1 = oldEdge->getEndPoint(FirstPoint);
    vertex2 = oldEdge->getEndPoint(SecondPoint);
    vertices[0] = vertex1;
    vertices[1] = vertex2;
    vertices[2] = newVertex;
    orient = Sphere::orient(vertex1, vertex2, newVertex);
    if (crossedEdge != NULL)
        *crossedEdge = NULL;
    // -------------------------------------------------------------------------
    // branch-1:
    // check the two new edges will not cross by any vertex
    // =========================================================================
    // paired vertices of old edge
    itVtx = oldEdge->detectAgent.vertices.begin();
    while (itVtx != oldEdge->detectAgent.vertices.end()) {
        // test point will be checked later
        if ((*itVtx)->getID() == -1 || *itVtx == oldVertex) {
            itVtx++;
            continue;
        }
        projection = (*itVtx)->detectAgent.getProjection(oldEdge);
        linkedEdge = (*itVtx)->linkedEdges.front();
        while (linkedEdge != NULL) {
            edge3 = linkedEdge->edge;
            if (find(checkedEdges.begin(), checkedEdges.end(), edge3)
                == checkedEdges.end()) {
                vertex3 = edge3->getEndPoint(FirstPoint);
                vertex4 = edge3->getEndPoint(SecondPoint);
                if (projection->getOrient() != orient && orient != OrientOn &&
                    oldVertex != NULL &&
                    oldVertex->detectAgent.getProjection(edge3) != NULL) {
                    if (vertex3 != oldVertex && vertex4 != oldVertex &&
                        Sphere::isIntersect(vertex3, vertex4, oldVertex, newVertex))
                        goto return_insert_vertex_cross_edge;
                } else {
                    if (vertex3 != oldVertex && vertex4 != oldVertex) {
                        if ((vertex1 != vertex3 && vertex1 != vertex4 &&
                             Sphere::isIntersect(vertex1, newVertex, vertex3, vertex4)) ||
                            (vertex2 != vertex3 && vertex2 != vertex4 &&
                             Sphere::isIntersect(vertex2, newVertex, vertex3, vertex4)) ||
                            (vertex1 == vertex3 &&
                             Sphere::orient(vertex1, newVertex, vertex4) !=
                             projection->getOrient()) ||
                            (vertex1 == vertex4 &&
                             Sphere::orient(vertex1, newVertex, vertex3) !=
                             projection->getOrient()) ||
                            (vertex2 == vertex3 &&
                             Sphere::orient(newVertex, vertex2, vertex4) !=
                             projection->getOrient()) ||
                            (vertex2 == vertex4 &&
                             Sphere::orient(newVertex, vertex2, vertex3) !=
                             projection->getOrient()))
                            goto return_insert_vertex_cross_edge;
                    }
                }
                checkedEdges.push_back(edge3);
            }
            linkedEdge = linkedEdge->next;
        }
        itVtx++;
    }
    // =========================================================================
    // paired edges of end points of old edge
    for (int i = 0; i < 3; ++i) {
        itPrj = vertices[i]->detectAgent.getProjections().begin();
        while (itPrj != vertices[i]->detectAgent.getProjections().end()) {
            edge3 = (*itPrj).getEdge();
            if (find(checkedEdges.begin(), checkedEdges.end(), edge3)
                != checkedEdges.end()) {
                itPrj++;
                continue;
            }
            vertex3 = edge3->getEndPoint(FirstPoint);
            vertex4 = edge3->getEndPoint(SecondPoint);
            if (vertex3 == oldVertex || vertex4 == oldVertex) {
                itPrj++;
                continue;
            }
            if ((vertex1 != vertex3 && vertex1 != vertex4 &&
                 Sphere::isIntersect(vertex1, newVertex, vertex3, vertex4)) ||
                (vertex2 != vertex3 && vertex2 != vertex4 &&
                 Sphere::isIntersect(vertex2, newVertex, vertex3, vertex4)))
                goto return_insert_vertex_cross_edge;
            checkedEdges.push_back(edge3);
            itPrj++;
        }
    }
    // -------------------------------------------------------------------------
    // branch-2:
    // check the two new edges will not cross by any test point, if so reset it
    itVtx = oldEdge->detectAgent.vertices.begin();
    while (itVtx != oldEdge->detectAgent.vertices.end()) {
        if ((*itVtx)->getID() != -1) {
            itVtx++;
            continue;
        }
        TestPoint *testPoint = static_cast<TestPoint *>(*itVtx);
        edge3 = testPoint->getHostEdge();
        vertex3 = edge3->getEndPoint(FirstPoint);
        vertex4 = edge3->getEndPoint(SecondPoint);
        if ((vertex3 != oldVertex && vertex4 != oldVertex) &&
            ((vertex1 != vertex3 &&
              Sphere::isIntersect(vertex1, newVertex, vertex3, testPoint)) ||
             (vertex1 != vertex4 &&
              Sphere::isIntersect(vertex1, newVertex, vertex4, testPoint)) ||
             (vertex2 != vertex3 &&
              Sphere::isIntersect(vertex2, newVertex, vertex3, testPoint)) ||
             (vertex2 != vertex4 &&
              Sphere::isIntersect(vertex2, newVertex, vertex4, testPoint)))) {
            itVtx++;
            testPoint->reset(meshManager);
        } else
            itVtx++;
    }
    return NoCross;
return_insert_vertex_cross_edge:
    if (crossedEdge != NULL && vertex3 != oldVertex && vertex4 != oldVertex)
        *crossedEdge = edge3;
    return Cross;
}

Status PotentialCrossDetector::detectRemoveVertexOnEdges(MeshManager &meshManager,
                                                         EdgePointer *edgePointer,
                                                         Vertex *testPoint,
                                                         Polygon *polygon)
{
    // -------------------------------------------------------------------------
    Vertex *vertex1, *vertex2, *vertex3, *vertex4, *vertex5, *vertex6, *vertices[3];
    Edge *edge1, *edge2, *edge;
    EdgePointer *linkedEdge;
    Polygon *markPolygon;
    static std::list<Edge *> checkedEdges;
    checkedEdges.clear();
    static std::list<Vertex *>::const_iterator itVtx;
    static std::list<Projection>::const_iterator itPrj;
    OrientStatus orient;
    Projection *projection;
    int mode;
    // -------------------------------------------------------------------------
    // collect information
    edge1 = edgePointer->prev->edge;
    edge2 = edgePointer->edge;
    vertex1 = edgePointer->prev->getEndPoint(FirstPoint);
    vertex2 = edgePointer->getEndPoint(FirstPoint);
    vertex3 = edgePointer->getEndPoint(SecondPoint);
    vertices[0] = vertex1;
    vertices[1] = vertex2;
    vertices[2] = vertex3;
    orient = Sphere::orient(vertex1, vertex3, vertex2);
    if (edgePointer->prev->orient == OrientLeft &&
        edgePointer->orient == OrientLeft)
        mode = 1;
    else if (edgePointer->prev->orient == OrientRight &&
             edgePointer->orient == OrientLeft)
        mode = 2;
    else if (edgePointer->prev->orient == OrientLeft &&
             edgePointer->orient == OrientRight)
        mode = 3;
    else if (edgePointer->prev->orient == OrientRight &&
             edgePointer->orient == OrientRight)
        mode = 4;
    // -------------------------------------------------------------------------
    // branch-1:
    // check the new edge will not be crossed by the paired vertices of edge 1/2
    // =========================================================================
    // edge 1
    for (itVtx = edge1->detectAgent.vertices.begin();
         itVtx != edge1->detectAgent.vertices.end(); ++itVtx) {
        projection = (*itVtx)->detectAgent.getProjection(edge1);
        if (mode == 1 || mode == 3) {
            if ((orient == OrientOn || projection->getOrient() != orient) &&
                projection->getOrient() != Sphere::orient(vertex1, vertex3, *itVtx))
                return Cross;
        } else {
            if ((orient == OrientOn || projection->getOrient() == orient) &&
                projection->getOrient() == Sphere::orient(vertex1, vertex3, *itVtx))
                return Cross;
        }
    }
    // =========================================================================
    // edge2
    for (itVtx = edge2->detectAgent.vertices.begin();
         itVtx != edge2->detectAgent.vertices.end(); ++itVtx) {
        projection = (*itVtx)->detectAgent.getProjection(edge2);
        if (mode == 1 || mode == 2) {
            if ((orient == OrientOn || projection->getOrient() != orient) &&
                projection->getOrient() != Sphere::orient(vertex1, vertex3, *itVtx))
                return Cross;
        } else {
            if ((orient == OrientOn || projection->getOrient() == orient) &&
                projection->getOrient() == Sphere::orient(vertex1, vertex3, *itVtx))
                return Cross;
        }
    }
    // -------------------------------------------------------------------------
    // branch-2:
    // check the new test point will not cross any edge, if so reset it
    orient = Sphere::orient(vertex1, vertex3, testPoint);
    if (mode == 1 || mode == 2) {
        if (orient == OrientLeft)
            markPolygon = polygon;
        else
            markPolygon = edgePointer->getPolygon(OrientRight);
    } else {
        if (orient == OrientLeft)
            markPolygon = edgePointer->getPolygon(OrientRight);
        else
            markPolygon = polygon;
    }
    // =========================================================================
    // edge 1
    for (itVtx = edge1->detectAgent.vertices.begin();
         itVtx != edge1->detectAgent.vertices.end(); ++itVtx) {
        vertex4 = *itVtx;
        projection = vertex4->detectAgent.getProjection(edge1);
        if (((mode == 1 || mode == 3) && projection->getOrient() != orient) ||
            ((mode == 2 || mode == 4) && projection->getOrient() == orient))
            continue;
        if (vertex4->getID() == -1)
            continue;
        linkedEdge = vertex4->linkedEdges.front();
        while (linkedEdge != NULL) {
            edge = linkedEdge->edge;
            if (find(checkedEdges.begin(), checkedEdges.end(), edge)
                == checkedEdges.end()) {
                vertex5 = NULL; vertex6 = NULL;
                if (edge->getPolygon(OrientLeft) == markPolygon ||
                    edge->getPolygon(OrientRight) == markPolygon) {
                    vertex5 = edge->getEndPoint(FirstPoint);
                    vertex6 = edge->getEndPoint(SecondPoint);
                }
                if (vertex5 != NULL) {
                    if (((vertex5 != vertex1 && vertex6 != vertex1) &&
                         Sphere::isIntersect(vertex1, testPoint, vertex5, vertex6)) ||
                        ((vertex5 != vertex3 && vertex6 != vertex3) &&
                         Sphere::isIntersect(vertex3, testPoint, vertex5, vertex6)))
                        goto return_nocross_but_reset_testpoint;
                }
                checkedEdges.push_back(edge);
            }
            linkedEdge = linkedEdge->next;
        }
    }
    // =========================================================================
    // edge 2
    for (itVtx = edge2->detectAgent.vertices.begin();
         itVtx != edge2->detectAgent.vertices.end(); ++itVtx) {
        vertex4 = *itVtx;
        projection = vertex4->detectAgent.getProjection(edge2);
        if (((mode == 1 || mode == 2) && projection->getOrient() != orient) ||
            ((mode == 3 || mode == 4) && projection->getOrient() == orient))
            continue;
        if (vertex4->getID() == -1)
            continue;
        linkedEdge = vertex4->linkedEdges.front();
        while (linkedEdge != NULL) {
            edge = linkedEdge->edge;
            if (find(checkedEdges.begin(), checkedEdges.end(), edge)
                == checkedEdges.end()) {
                vertex5 = NULL; vertex6 = NULL;
                if (edge->getPolygon(OrientLeft) == markPolygon ||
                    edge->getPolygon(OrientRight) == markPolygon) {
                    vertex5 = edge->getEndPoint(FirstPoint);
                    vertex6 = edge->getEndPoint(SecondPoint);
                }
                if (vertex5 != NULL) {
                    if (((vertex5 != vertex1 && vertex6 != vertex1) &&
                         Sphere::isIntersect(vertex1, testPoint, vertex5, vertex6)) ||
                        ((vertex5 != vertex3 && vertex6 != vertex3) &&
                         Sphere::isIntersect(vertex3, testPoint, vertex5, vertex6)))
                        goto return_nocross_but_reset_testpoint;
                }
                checkedEdges.push_back(edge);
            }
            linkedEdge = linkedEdge->next;
        }
    }
    // =========================================================================
    for (int i = 0; i < 3; ++i) {
        for (itPrj = vertices[i]->detectAgent.getProjections().begin();
             itPrj != vertices[i]->detectAgent.getProjections().end(); ++itPrj) {
            edge = (*itPrj).getEdge();
            if (find(checkedEdges.begin(), checkedEdges.end(), edge)
                == checkedEdges.end()) {
                vertex5 = NULL; vertex6 = NULL;
                if (edge->getPolygon(OrientLeft) == markPolygon ||
                    edge->getPolygon(OrientRight) == markPolygon) {
                    vertex5 = edge->getEndPoint(FirstPoint);
                    vertex6 = edge->getEndPoint(SecondPoint);
                }
                if (vertex5 != NULL) {
                    if (((vertex5 != vertex1 && vertex6 != vertex1) &&
                         Sphere::isIntersect(vertex1, testPoint, vertex5, vertex6)) ||
                        ((vertex5 != vertex3 && vertex6 != vertex3) &&
                         Sphere::isIntersect(vertex3, testPoint, vertex5, vertex6)))
                        goto return_nocross_but_reset_testpoint;
                }
                checkedEdges.push_back(edge);
            }
        }
    }
    // -------------------------------------------------------------------------
    return NoCross;
return_nocross_but_reset_testpoint:
    Coordinate x1, x2; Location loc;
    x1 = testPoint->getCoordinate(OldTimeLevel);
    Sphere::calcMiddlePoint(vertex1->getCoordinate(NewTimeLevel),
                            vertex3->getCoordinate(NewTimeLevel), x2);
    meshManager.checkLocation(x2, loc);
    testPoint->setCoordinate(x2, NewTimeLevel);
    testPoint->setLocation(loc);
    testPoint->setCoordinate(x1, OldTimeLevel);
    return NoCross;
}

Status PotentialCrossDetector::detectAddConnection(Polygon *polygon,
                                                   EdgePointer *edgePointer1,
                                                   EdgePointer *&edgePointer2,
                                                   bool &isConnectOk1,
                                                   bool &isConnectOk2,
                                                   Vector &vector1,
                                                   Vector &vector2)
{
    Vertex *vertex1, *vertex3, *vertex4, *vertex5, *vertex6;
    EdgePointer *edgePointer;
    static std::list<Vertex *>::const_iterator itVtx;
    static std::list<Projection>::const_iterator itPrj;
    double angle;

    // -------------------------------------------------------------------------
    // collect information
    vertex1 = edgePointer1->getEndPoint(FirstPoint);
    vertex3 = edgePointer2->getEndPoint(FirstPoint);
    vertex4 = edgePointer2->getEndPoint(SecondPoint);
    isConnectOk1 = true;
    isConnectOk2 = true;
    // -------------------------------------------------------------------------
    edgePointer = edgePointer1->next;
    while (edgePointer != edgePointer2) {
        vertex5 = edgePointer->edge->getEndPoint(FirstPoint);
        vertex6 = edgePointer->edge->getEndPoint(SecondPoint);
        if (vertex5 == vertex1 ||  vertex6 == vertex1) {
            edgePointer = edgePointer->next;
            continue;
        }
        if (isConnectOk1 && vertex5 != vertex3 && vertex6 != vertex3 &&
            Sphere::isIntersect(vertex1, vertex3, vertex5, vertex6))
            isConnectOk1 = false;
        if (isConnectOk2 && vertex5 != vertex4 && vertex6 != vertex4 &&
            Sphere::isIntersect(vertex1, vertex4, vertex5, vertex6))
            isConnectOk2 = false;
        if (!isConnectOk1 && !isConnectOk2)
            return Cross;
        edgePointer = edgePointer->next;
    }
    edgePointer = edgePointer1->prev;
    while (edgePointer != edgePointer2) {
        vertex5 = edgePointer->edge->getEndPoint(FirstPoint);
        vertex6 = edgePointer->edge->getEndPoint(SecondPoint);
        if (vertex5 == vertex1 ||  vertex6 == vertex1) {
            edgePointer = edgePointer->prev;
            continue;
        }
        if (isConnectOk1 && vertex5 != vertex3 && vertex6 != vertex3 &&
            Sphere::isIntersect(vertex1, vertex3, vertex5, vertex6))
            isConnectOk1 = false;
        if (isConnectOk2 && vertex5 != vertex4 && vertex6 != vertex4 &&
            Sphere::isIntersect(vertex1, vertex4, vertex5, vertex6))
            isConnectOk2 = false;
        if (!isConnectOk1 && !isConnectOk2)
            return Cross;
        edgePointer = edgePointer->prev;
    }
    // -------------------------------------------------------------------------
    if (isConnectOk1) {
        vector1 = norm_cross(vertex1->getCoordinate().getCAR(),
                             vertex3->getCoordinate().getCAR());
        angle = EdgePointer::calcAngle
        (-vector1, edgePointer2->getNormVector(), *vertex3);
        if (angle >= edgePointer2->getAngle())
            isConnectOk1 = false;
    }
    if (isConnectOk2) {
        vector2 = norm_cross(vertex1->getCoordinate().getCAR(),
                             vertex4->getCoordinate().getCAR());
        angle = EdgePointer::calcAngle
        (edgePointer2->getNormVector(), vector2, *vertex4);
        if (angle >= edgePointer2->next->getAngle())
            isConnectOk2 = false;
    }
    if (!isConnectOk1 && !isConnectOk2)
        return Cross;
    // -------------------------------------------------------------------------
    if (isConnectOk1)
        if (Sphere::isIntersect(vertex1, vertex3,
                                edgePointer2->edge->getTestPoint(), vertex4))
            isConnectOk1 = false;
    if (isConnectOk2)
        if (Sphere::isIntersect(vertex1, vertex4,
                                edgePointer2->edge->getTestPoint(), vertex3))
            isConnectOk2 = false;
    if (!isConnectOk1 && !isConnectOk2)
        return Cross;
    // -------------------------------------------------------------------------
    return NoCross;
}

Status PotentialCrossDetector::detectTestPoint(EdgePointer *edgePointer1,
                                               EdgePointer *edgePointer2)
{
    Vertex *vertex1, *vertex2, *vertex3, *vertex4;
    TestPoint *testPoint;
    // -------------------------------------------------------------------------
    // collect information
    vertex1 = edgePointer1->edge->getEndPoint(FirstPoint);
    vertex2 = edgePointer1->edge->getEndPoint(SecondPoint);
    vertex3 = edgePointer2->edge->getEndPoint(FirstPoint);
    vertex4 = edgePointer2->edge->getEndPoint(SecondPoint);
    testPoint = edgePointer2->edge->getTestPoint();
    // -------------------------------------------------------------------------
    if ((vertex3 != vertex1 && vertex3 != vertex2 &&
         Sphere::isIntersect(vertex1, vertex2, vertex3, testPoint)) ||
        (vertex4 != vertex1 && vertex4 != vertex2 &&
         Sphere::isIntersect(vertex1, vertex2, vertex4, testPoint)))
        return Cross;
    // -------------------------------------------------------------------------
    return NoCross;
}

// TODO: Could this function be used by others to avoid duplicate?
Status PotentialCrossDetector::detectVertex(Vertex *vertex, Edge *edge)
{
    Vertex *vertex1, *vertex2, *vertex3, *vertex4;
    vertex1 = edge->getEndPoint(FirstPoint);
    vertex2 = edge->getEndPoint(SecondPoint);
    EdgePointer *linkedEdge = vertex->linkedEdges.front();
    for (int i = 0; i < vertex->linkedEdges.size(); ++i) {
        vertex3 = linkedEdge->edge->getEndPoint(FirstPoint);
        vertex4 = linkedEdge->edge->getEndPoint(SecondPoint);
        if (Sphere::isIntersect(vertex1, vertex2, vertex3, vertex4))
            return Cross;
        linkedEdge = linkedEdge->next;
    }
    return NoCross;
}

void PotentialCrossDetector::adjustMergeEdgeAngleThreshold(Edge *edge1,
                                                           Edge *edge2,
                                                           double &a0)
{
    static const double smallDistance = 0.1/Rad2Deg*Sphere::radius;
    Vertex *vertex;
    std::list<Vertex *>::const_iterator itVtx;
    for (itVtx = edge1->detectAgent.vertices.begin();
         itVtx != edge1->detectAgent.vertices.end(); ++itVtx) {
        vertex = *itVtx;
        Projection *projection = vertex->detectAgent.getProjection(edge1);
        if (projection->getDistance(NewTimeLevel) < smallDistance ||
            projection->getChangeRate() > 0.3) {
            a0 *= 0.2;
            return;
        }
    }
    for (itVtx = edge2->detectAgent.vertices.begin();
         itVtx != edge2->detectAgent.vertices.end(); ++itVtx) {
        vertex = *itVtx;
        Projection *projection = vertex->detectAgent.getProjection(edge2);
        if (projection->getDistance(NewTimeLevel) < smallDistance ||
            projection->getChangeRate() > 0.3) {
            a0 *= 0.2;
            return;
        }
    }
    if (edge1->getEndPoint(SecondPoint) == edge2->getEndPoint(FirstPoint) ||
        edge1->getEndPoint(SecondPoint) == edge2->getEndPoint(SecondPoint))
        vertex = edge1->getEndPoint(SecondPoint);
    else if (edge1->getEndPoint(FirstPoint) == edge2->getEndPoint(FirstPoint) ||
             edge1->getEndPoint(FirstPoint) == edge2->getEndPoint(SecondPoint))
        vertex = edge1->getEndPoint(FirstPoint);
    std::list<Projection>::const_iterator itPrj;
    for (itPrj = vertex->detectAgent.getProjections().begin();
         itPrj != vertex->detectAgent.getProjections().end(); ++itPrj) {
        if ((*itPrj).getDistance(NewTimeLevel) < smallDistance ||
            (*itPrj).getChangeRate() > 0.3) {
            a0 *= 0.2;
            return;
        }
    }
}