#include "PotentialCrossDetector.h"
#include "ApproachDetector.h"
#include "CurvatureGuard.h"
#include "Vertex.h"
#include "Edge.h"
#include "MeshManager.h"

using namespace PotentialCrossDetector;
using namespace ApproachDetector;
using namespace CurvatureGuard;

Status PotentialCrossDetector::detectReplaceVertex(Vertex *oldVertex,
                                                   Vertex *newVertex,
                                                   bool isJustDetect)
{
    std::list<Vertex *>::const_iterator itVtx;
    Vertex *vertex1, *vertex2, *vertex3;
    Projection *projection;
    OrientStatus orient;
    EdgePointer *linkedEdge = oldVertex->linkedEdges.front();
    for (int i = 0; i < oldVertex->linkedEdges.size(); ++i) {
        Edge *edge = linkedEdge->edge;
        vertex1 = NULL, vertex2 = NULL;
        if (edge->getEndPoint(FirstPoint) == oldVertex) {
            vertex1 = newVertex;
            vertex2 = edge->getEndPoint(SecondPoint);
        } else if (edge->getEndPoint(SecondPoint) == oldVertex) {
            vertex1 = edge->getEndPoint(FirstPoint);
            vertex2 = newVertex;
        }
        if (vertex1 != vertex2)
            for (itVtx = edge->detectAgent.vertices.begin();
                 itVtx != edge->detectAgent.vertices.end(); ++itVtx) {
                vertex3 = *itVtx;
                if (vertex3->getID() == -1 ||
                    vertex3 == vertex1 || vertex3 == vertex2)
                    continue;
                projection = vertex3->detectAgent.getProjection(edge);
                orient = Sphere::orient(vertex1, vertex2, vertex3);
                if (orient != projection->getOrient()) {
                    if (!isJustDetect && projection->isApproaching())
                        ApproachingVertices::jumpVertex(oldVertex, vertex3);
                    return Cross;
                }
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
    Vertex *vertex1, *vertex2, *vertex3, *vertex4, *vertices[2];
    OrientStatus orient;
    Polygon *markPolygon;
    std::list<Projection>::iterator itPrj;
    std::list<Vertex *>::iterator itVtx;
    Projection *projection;
    EdgePointer *linkedEdge, *nextLinkedEdge;
    Edge *edge3;
    bool isEdgeSplit;
    static std::list<Edge *> checkedEdges;
    checkedEdges.clear();
    // -------------------------------------------------------------------------
    // collect information
    vertex1 = oldEdge->getEndPoint(FirstPoint);
    vertex2 = oldEdge->getEndPoint(SecondPoint);
    vertices[0] = vertex1;
    vertices[1] = vertex2;
    orient = Sphere::orient(vertex1, vertex2, newVertex);
    if (orient != OrientOn)
        markPolygon = oldEdge->getPolygon(orient);
    else
        markPolygon = NULL;
    if (crossedEdge != NULL)
        *crossedEdge = NULL;
    // -------------------------------------------------------------------------
    // check the paired vertices of oldEdge to find out close edges
    itVtx = oldEdge->detectAgent.vertices.begin();
    while (itVtx != oldEdge->detectAgent.vertices.end()) {
        // test point will be checked later
        if ((*itVtx)->getID() == -1 || *itVtx == oldVertex) {
            itVtx++;
            continue;
        }
        projection = (*itVtx)->detectAgent.getProjection(oldEdge);
        // only check the same side
        if (projection->getOrient() != orient && orient != OrientOn) {
            itVtx++;
            continue;
        }
        // linked edge may be split, if this happens, recheck the paired
        // vertices
        isEdgeSplit = false;
        linkedEdge = (*itVtx)->linkedEdges.front();
        while (linkedEdge != NULL) {
            nextLinkedEdge = linkedEdge->next;
            edge3 = linkedEdge->edge;
            if (find(checkedEdges.begin(), checkedEdges.end(), edge3)
                == checkedEdges.end()) {
                vertex3 = NULL; vertex4 = NULL;
                if (edge3->getPolygon(OrientLeft) == markPolygon ||
                    edge3->getPolygon(OrientRight) == markPolygon ||
                    markPolygon == NULL) {
                    vertex3 = edge3->getEndPoint(FirstPoint);
                    vertex4 = edge3->getEndPoint(SecondPoint);
                }
                if (vertex3 != NULL && vertex3 != oldVertex && vertex4 != oldVertex) {
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
                    checkedEdges.push_back(edge3);
                }
            }
            linkedEdge = nextLinkedEdge;
        }
        if (isEdgeSplit)
            itVtx = oldEdge->detectAgent.vertices.begin();
        else
            itVtx++;
    }
    // -------------------------------------------------------------------------
    // check the paired edges of the end points of oldEdge
    for (int i = 0; i < 2; ++i) {
        itPrj = vertices[i]->detectAgent.getProjections().begin();
        while (itPrj != vertices[i]->detectAgent.getProjections().end()) {
            isEdgeSplit = false;
            edge3 = (*itPrj).getEdge();
            if (find(checkedEdges.begin(), checkedEdges.end(), edge3)
                != checkedEdges.end()) {
                itPrj++;
                continue;
            }
            vertex3 = NULL; vertex4 = NULL;
            if (edge3->getPolygon(OrientLeft) == markPolygon ||
                edge3->getPolygon(OrientRight) == markPolygon) {
                vertex3 = edge3->getEndPoint(FirstPoint);
                vertex4 = edge3->getEndPoint(SecondPoint);
            }
            if (vertex3 == oldVertex || vertex4 == oldVertex) {
                itPrj++;
                continue;
            }
            if (vertex3 != NULL) {
                if ((vertex1 != vertex3 && vertex1 != vertex4 &&
    				 Sphere::isIntersect(vertex1, newVertex, vertex3, vertex4)) ||
                    (vertex2 != vertex3 && vertex2 != vertex4 &&
                     Sphere::isIntersect(vertex2, newVertex, vertex3, vertex4)))
                    goto return_insert_vertex_cross_edge;
                checkedEdges.push_back(edge3);
            }
            if (isEdgeSplit)
                itPrj = vertices[i]->detectAgent.getProjections().begin();
            else
                itPrj++;
        }
    }
    // -------------------------------------------------------------------------
    // check test point, if it crosses the edge, reset it
    for (itVtx = oldEdge->detectAgent.vertices.begin();
         itVtx != oldEdge->detectAgent.vertices.end(); ++itVtx) {
        if ((*itVtx)->getID() != -1) continue;
        edge3 = (*itVtx)->getHostEdge();
        vertex3 = edge3->getEndPoint(FirstPoint);
        vertex4 = edge3->getEndPoint(SecondPoint);
        if ((vertex1 != vertex3 &&
             Sphere::isIntersect(vertex1, newVertex, vertex3, *itVtx)) ||
            (vertex1 != vertex4 &&
             Sphere::isIntersect(vertex1, newVertex, vertex4, *itVtx)) ||
            (vertex2 != vertex3 &&
             Sphere::isIntersect(vertex2, newVertex, vertex3, *itVtx)) ||
            (vertex2 != vertex4 &&
             Sphere::isIntersect(vertex2, newVertex, vertex4, *itVtx))) {
            edge3->getTestPoint()->reset(meshManager);
            // TODO: The following operations should be merged into reset
            //       function.
            projection = (*itVtx)->detectAgent.getProjection(oldEdge);
            if (projection->project(NewTimeLevel) == HasProjection) {
                projection->project(OldTimeLevel);
                projection->checkApproaching();
                if ((*itVtx)->detectAgent.getActiveProjection() == NULL)
                    ApproachingVertices::removeVertex(*itVtx);
            } else
                AgentPair::unpair(itVtx, projection);
        }
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
    Edge *edge1 = edgePointer->prev->edge;
    Edge *edge2 = edgePointer->edge;
    Vertex *vertex1 = edgePointer->prev->getEndPoint(FirstPoint);
    Vertex *vertex2 = edgePointer->getEndPoint(FirstPoint);
    Vertex *vertex3 = edgePointer->getEndPoint(SecondPoint);
    // -------------------------------------------------------------------------
    OrientStatus orient;
    // -------------------------------------------------------------------------
    int mode;
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
    // check the potential new edge will not be crossed by the paired vertices
    // of edge1 and edge2
    std::list<Vertex *>::const_iterator it;
    Projection *projection;
    orient = Sphere::orient(vertex1, vertex3, vertex2);
    // =========================================================================
    // edge 1
    for (it = edge1->detectAgent.vertices.begin();
         it != edge1->detectAgent.vertices.end(); ++it) {
        projection = (*it)->detectAgent.getProjection(edge1);
        if (mode == 1 || mode == 3) {
            if ((orient == OrientOn || projection->getOrient() != orient) &&
                projection->getOrient() != Sphere::orient(vertex1, vertex3, *it))
                return Cross;
        } else {
            if ((orient == OrientOn || projection->getOrient() == orient) &&
                projection->getOrient() == Sphere::orient(vertex1, vertex3, *it))
                return Cross;
        }
    }
    // =========================================================================
    // edge2
    for (it = edge2->detectAgent.vertices.begin();
         it != edge2->detectAgent.vertices.end(); ++it) {
        projection = (*it)->detectAgent.getProjection(edge2);
        if (mode == 1 || mode == 2) {
            if ((orient == OrientOn || projection->getOrient() != orient) &&
                projection->getOrient() != Sphere::orient(vertex1, vertex3, *it))
                return Cross;
        } else {
            if ((orient == OrientOn || projection->getOrient() == orient) &&
                projection->getOrient() == Sphere::orient(vertex1, vertex3, *it))
                return Cross;
        }
    }
    // -------------------------------------------------------------------------
    // check for test point, if it crosses any edge, reset it
    Vertex *vertex4, *vertex5, *vertex6;
    EdgePointer *linkedEdge;
    Edge *edge;
    Polygon *markPolygon;
    static std::list<Edge *> checkedEdges;
    checkedEdges.clear();
    orient = Sphere::orient(vertex1, vertex3, testPoint);
    if (mode == 1 || mode == 2) {
        if (orient == OrientLeft)
            markPolygon = markPolygon;
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
    for (it = edge1->detectAgent.vertices.begin();
         it != edge1->detectAgent.vertices.end(); ++it) {
        vertex4 = *it;
        projection = vertex4->detectAgent.getProjection(edge1);
        if (((mode == 1 || mode == 3) && projection->getOrient() != orient) ||
            ((mode == 2 || mode == 4) && projection->getOrient() == orient))
            continue;
        if (vertex4->getID() != -1) {
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
        // TODO: Clean the codes.
//        else {
//            edge = vertex4->getHostEdge();
//            if (edge == edge2)
//                continue;
//            vertex5 = edge->getEndPoint(FirstPoint);
//            vertex6 = edge->getEndPoint(SecondPoint);
//            if ((vertex1 != vertex5 &&
//                 Sphere::isIntersect(vertex1, testPoint, vertex5, *it)) ||
//                (vertex1 != vertex6 &&
//                 Sphere::isIntersect(vertex1, testPoint, vertex6, *it)) ||
//                (vertex3 != vertex5 &&
//                 Sphere::isIntersect(vertex3, testPoint, vertex5, *it)) ||
//                (vertex3 != vertex6 &&
//                 Sphere::isIntersect(vertex3, testPoint, vertex6, *it)))
//                goto return_nocross_but_reset_testpoint;
//        }
    }
    // =========================================================================
    // edge 2
    for (it = edge2->detectAgent.vertices.begin();
         it != edge2->detectAgent.vertices.end(); ++it) {
        vertex4 = *it;
        projection = vertex4->detectAgent.getProjection(edge2);
        if (((mode == 1 || mode == 2) && projection->getOrient() != orient) ||
            ((mode == 3 || mode == 4) && projection->getOrient() == orient))
            continue;
        if (vertex4->getID() != -1) {
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
//        else {
//            edge = vertex4->getHostEdge();
//            if (edge == edge1)
//                continue;
//                vertex5 = edge->getEndPoint(FirstPoint);
//                vertex6 = edge->getEndPoint(SecondPoint);
//                if ((vertex1 != vertex5 &&
//                     Sphere::isIntersect(vertex1, testPoint, vertex5, *it)) ||
//                    (vertex1 != vertex6 &&
//                     Sphere::isIntersect(vertex1, testPoint, vertex6, *it)) ||
//                    (vertex3 != vertex5 &&
//                     Sphere::isIntersect(vertex3, testPoint, vertex5, *it)) ||
//                    (vertex3 != vertex6 &&
//                     Sphere::isIntersect(vertex3, testPoint, vertex6, *it)))
//                    goto return_nocross_but_reset_testpoint;
//        }
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
    Vertex *vertex1, *vertex3, *vertex4, *vertex5, *vertex6, *vertex7;
    Edge *edge2;
    EdgePointer *linkedEdge;
    Projection *projection1, *projection2;
    static std::list<Vertex *>::const_iterator itVtx;
    static std::list<Projection>::const_iterator itPrj;
    double angle;

    // -------------------------------------------------------------------------
    // collect information
    vertex1 = edgePointer1->getEndPoint(FirstPoint);
    vertex3 = edgePointer2->getEndPoint(FirstPoint);
    vertex4 = edgePointer2->getEndPoint(SecondPoint);
    edge2 = edgePointer2->edge;
    projection1 = vertex1->detectAgent.getProjection(edge2);
    isConnectOk1 = true;
    isConnectOk2 = true;
    // -------------------------------------------------------------------------
    // check the paired edges of vertex 1
    for (itPrj = vertex1->detectAgent.getProjections().begin();
         itPrj != vertex1->detectAgent.getProjections().end(); ++itPrj) {
        // condition-1
        if ((*itPrj).getEdge() == edge2)
            continue;
        vertex6 = (*itPrj).getEdge()->getEndPoint(FirstPoint);
        vertex7 = (*itPrj).getEdge()->getEndPoint(SecondPoint);
        if (isConnectOk1 && vertex6 != vertex3 && vertex7 != vertex3 &&
            Sphere::isIntersect(vertex1, vertex3, vertex6, vertex7))
            isConnectOk1 = false;
        if (isConnectOk2 && vertex6 != vertex4 && vertex7 != vertex4 &&
            Sphere::isIntersect(vertex1, vertex4, vertex6, vertex7))
            isConnectOk2 = false;
        if (!isConnectOk1 && !isConnectOk2)
            return Cross;
    }
    // -------------------------------------------------------------------------
    // check the paired vertices of edge 2
    for (itVtx = edge2->detectAgent.vertices.begin();
         itVtx != edge2->detectAgent.vertices.end(); ++itVtx) {
        vertex5 = *itVtx;
        // condition-1
        if (vertex5 == vertex1) continue;
        projection2 = vertex5->detectAgent.getProjection(edge2);
        // condition-2
        if (projection1->getDistance() < projection2->getDistance() ||
            projection1->getOrient() != projection2->getOrient()) continue;
        // condition-3
        if (vertex5->getID() != -1) {
            linkedEdge = vertex5->linkedEdges.front();
            for (int i = 0; i < vertex5->linkedEdges.size(); ++i) {
                vertex6 = linkedEdge->edge->getEndPoint(FirstPoint);
                vertex7 = linkedEdge->edge->getEndPoint(SecondPoint);
                // condition-3-1
                if (vertex6 == vertex1 || vertex7 == vertex1) {
                    linkedEdge = linkedEdge->next;
                    continue;
                }
                if (isConnectOk1 && vertex6 != vertex3 && vertex7 != vertex3 &&
                    Sphere::isIntersect(vertex1, vertex3, vertex6, vertex7))
                    isConnectOk1 = false;
                if (isConnectOk2 && vertex6 != vertex4 && vertex7 != vertex4 &&
                    Sphere::isIntersect(vertex1, vertex4, vertex6, vertex7))
                    isConnectOk2 = false;
                if (!isConnectOk1 && !isConnectOk2)
                    return Cross;
                linkedEdge = linkedEdge->next;
            }
        }
        // condition-4
        else {
            vertex6 = vertex5->getHostEdge()->getEndPoint(FirstPoint);
            vertex7 = vertex5->getHostEdge()->getEndPoint(SecondPoint);
            // condition-4-1
            if (vertex6 != vertex1) {
                if (isConnectOk1 && vertex6 != vertex3 &&
                    Sphere::isIntersect(vertex1, vertex3, vertex5, vertex6))
                    isConnectOk1 = false;
                if (isConnectOk2 && vertex6 != vertex4 &&
                    Sphere::isIntersect(vertex1, vertex4, vertex5, vertex6))
                    isConnectOk2 = false;
            }
            // condition-4-2
            if (vertex7 != vertex1) {
                if (isConnectOk1 && vertex7 != vertex3 &&
                    Sphere::isIntersect(vertex1, vertex3, vertex5, vertex7))
                    isConnectOk1 = false;
                if (isConnectOk2 && vertex7 != vertex4 &&
                    Sphere::isIntersect(vertex1, vertex4, vertex5, vertex7))
                    isConnectOk2 = false;
            }
        }
        if (!isConnectOk1 && !isConnectOk2)
            return Cross;
    }
    // -------------------------------------------------------------------------
    // check the paired edges of edge 2
    if (isConnectOk1) {
        for (itPrj = vertex3->detectAgent.getProjections().begin();
             itPrj != vertex3->detectAgent.getProjections().end(); ++itPrj) {
            vertex6 = (*itPrj).getEdge()->getEndPoint(FirstPoint);
            vertex7 = (*itPrj).getEdge()->getEndPoint(SecondPoint);
            if (vertex6 == vertex1 || vertex7 == vertex1) continue;
            if (Sphere::isIntersect(vertex1, vertex3, vertex6, vertex7)) {
                isConnectOk1 = false;
                break;
            }
        }
    }
    if (isConnectOk2) {
        for (itPrj = vertex4->detectAgent.getProjections().begin();
             itPrj != vertex4->detectAgent.getProjections().end(); ++itPrj) {
            vertex6 = (*itPrj).getEdge()->getEndPoint(FirstPoint);
            vertex7 = (*itPrj).getEdge()->getEndPoint(SecondPoint);
            if (vertex6 == vertex1 || vertex7 == vertex1) continue;
            if (Sphere::isIntersect(vertex1, vertex4, vertex6, vertex7)) {
                isConnectOk2 = false;
                break;
            }
        }
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
    // -------------------------------------------------------------------------
    if (isConnectOk1)
        if (Sphere::isIntersect(vertex1, vertex3, edge2->getTestPoint(), vertex4))
            isConnectOk1 = false;
    if (isConnectOk2)
        if (Sphere::isIntersect(vertex1, vertex4, edge2->getTestPoint(), vertex3))
            isConnectOk2 = false;
    // -------------------------------------------------------------------------
    if (!isConnectOk1 && !isConnectOk2)
        return Cross;
    else
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