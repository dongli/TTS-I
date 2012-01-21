#ifndef splitPolygon_h
#define splitPolygon_h

#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "ApproachDetector.h"
#include "ApproachingVertices.h"
#include "PotentialCrossDetector.h"
#include "SpecialPolygons.h"
#include "CurvatureGuard.h"
#include "TTS.h"
#include "CommonTasks.h"
#ifdef DEBUG
#include "DebugTools.h"
#endif

using namespace ApproachDetector;
using namespace PotentialCrossDetector;
using namespace SpecialPolygons;
using namespace CurvatureGuard;

void CurvatureGuard::splitPolygon
(MeshManager &meshManager, const FlowManager &flowManager,
 PolygonManager &polygonManager, Polygon *&polygon1, EdgePointer *edgePointer1,
 EdgePointer *edgePointer2, Vertex *vertex3, int mode)
{
    Vertex *vertex1, *vertex2, *testVertex, vertex, *newVertex;
    Edge *edge1, *crossedEdge;
    Polygon *polygon2, *polygon3, *polygon4;
    EdgePointer *edgePointer3, *linkedEdge;
    Projection *projection;
    Location loc;
    OrientStatus orient;
    int i;

    // -------------------------------------------------------------------------
    // collect information
    edge1 = edgePointer1->edge;
    vertex1 = edgePointer1->getEndPoint(FirstPoint);
    vertex2 = edgePointer1->getEndPoint(SecondPoint);
    polygon2 = edgePointer1->getPolygon(OrientRight);
    projection = vertex3->detectAgent.getProjection(edge1);

    // -------------------------------------------------------------------------
    // set the test point
    switch (mode) {
        case 1:
            testVertex = vertex1;
            break;
        case 2:
            testVertex = vertex2;
            break;
        case 4:
            testVertex = &vertex;
            testVertex->setCoordinate(projection->getCoordinate(OldTimeLevel));
            meshManager.checkLocation(testVertex->getCoordinate(), loc);
            testVertex->setLocation(loc);
            TTS::track(meshManager, flowManager, testVertex);
            break;
        case 5:
            testVertex = vertex3;
            break;
        case 6:
            testVertex = &vertex;
            testVertex->setCoordinate
            (projection->getCoordinate(NewTimeLevel), NewTimeLevel);
            testVertex->setCoordinate
            (projection->getCoordinate(OldTimeLevel), OldTimeLevel);
            meshManager.checkLocation(testVertex->getCoordinate(), loc, testVertex);
            testVertex->setLocation(loc);
            break;
        default:
            REPORT_ERROR("Unknown mode!");
    }
    // -------------------------------------------------------------------------
    // check if the split operation is ok
    if (mode != 5 && detectReplaceVertex(edgePointer1, vertex3, testVertex) != NoCross) {
        if (mode == 1 || mode == 2) {
            projection->setApproach(false);
            if (vertex3->detectAgent.getActiveProjection() == NULL)
                ApproachingVertices::removeVertex(vertex3);
            return;
        }
        // when mode is 1/2/4, vertex3 will be eliminated, and this
        // may cause edge-crossing. If this happens, shift to mode 5
        // by making testVertex be vertex3.
        testVertex = vertex3;
        mode = 5;
        // TODO: Whether we should split the polygon at this condition?
    }
    if (mode == 4) {
        if (detectInsertVertexOnEdge(meshManager, flowManager, polygonManager,
                                     edge1, testVertex, vertex3,
                                     &crossedEdge) != NoCross) {
            testVertex->setCoordinate
            (projection->getCoordinate(NewTimeLevel), NewTimeLevel);
            testVertex->setCoordinate
            (projection->getCoordinate(OldTimeLevel), OldTimeLevel);
            meshManager.checkLocation(testVertex->getCoordinate(), loc, testVertex);
            testVertex->setLocation(loc);
        }
    } else {
        crossedEdge = NULL;
    }
    // -------------------------------------------------------------------------
    ApproachDetector::AgentPair::unpair(vertex3, edge1);
    if (vertex3->detectAgent.getActiveProjection() == NULL)
        ApproachingVertices::removeVertex(vertex3);
    // -------------------------------------------------------------------------
    // splitPolygon may affect the neightbor polygon, so record it for later
    // processing
    polygon4 = NULL;
    if (mode == 1 && edgePointer1 == edgePointer2->next->next)
        polygon4 = edgePointer2->next->getPolygon(OrientRight);
    else if (mode == 2 && edgePointer1 == edgePointer2->prev)
        polygon4 = edgePointer2->getPolygon(OrientRight);
    // -------------------------------------------------------------------------
    // create a new polygon
    polygonManager.polygons.append(&polygon3);
    edgePointer3 = edgePointer2->next;
    EdgePointer *endEdgePointer;
    if (mode == 1) {
        endEdgePointer = edgePointer1;
        newVertex = vertex1;
    } else if (mode == 2) {
        endEdgePointer = edgePointer1->next;
        newVertex = vertex2;
    } else {
        endEdgePointer = edgePointer1;
    }
    while (edgePointer3 != endEdgePointer) {
        EdgePointer *edgePointer;
        polygon3->edgePointers.append(&edgePointer);
        edgePointer->replace(edgePointer3);
        edgePointer->edge->setPolygon(edgePointer->orient, polygon3);
        edgePointer3 = edgePointer3->next;
        polygon1->edgePointers.remove(edgePointer3->prev);
    }
    // ---------------------------------------------------------------------
    Edge *newEdge = NULL;
    // check if the projection of vertex on the
    // edge 1 is too close to vertex 1 or 2
    // if true, just use the 1 or 2
    // if not, use the new one
    if (mode < 3) {
        polygon3->edgePointers.ring();
    } else {
        polygon3->edgePointers.append(&edgePointer3);
        polygon3->edgePointers.ring();
        // create a new vertex on the edge 1
        if (mode != 5) {
            polygonManager.vertices.append(&newVertex);
            *newVertex = *testVertex;
        } else
            newVertex = testVertex;
        // split the edge pointed by edge 1
        polygonManager.edges.append(&newEdge);
        newEdge->linkEndPoint(FirstPoint, vertex1);
        newEdge->linkEndPoint(SecondPoint, newVertex);
        newEdge->setPolygon(OrientLeft, polygon3);
        newEdge->setEdgePointer(OrientLeft, edgePointer3);
        newEdge->setPolygon(OrientRight, polygon2);
        EdgePointer *edgePointer5, *edgePointer6;
        if (edgePointer1->orient == OrientLeft) {
            edgePointer5 = edgePointer1->edge->getEdgePointer(OrientRight);
        } else {
            edgePointer5 = edgePointer1->edge->getEdgePointer(OrientLeft);
        }
        polygon2->edgePointers.insert(edgePointer5, &edgePointer6);
        newEdge->setEdgePointer(OrientRight, edgePointer6);
        newEdge->calcNormVector();
        newEdge->calcLength();
        Vertex *testPoint = newEdge->getTestPoint();
        meshManager.checkLocation(testPoint->getCoordinate(), loc);
        testPoint->setLocation(loc);
        TTS::track(meshManager, flowManager, testPoint);
    }
    // ---------------------------------------------------------------------
    // modify the old edges
    if (mode != 5) {
        // unpair the test point of edge1 from the linked edges of
        // vertex3 to avoid problems
        linkedEdge = vertex3->linkedEdges.front();
        for (i = 0; i < vertex3->linkedEdges.size(); ++i) {
            if (edge1->getTestPoint()->detectAgent.
                getProjection(linkedEdge->edge) != NULL)
                AgentPair::unpair(edge1->getTestPoint(), linkedEdge->edge);
            linkedEdge = linkedEdge->next;
        }
        vertex3->handoverEdges(newVertex, meshManager,
                               flowManager, polygonManager);
        polygonManager.vertices.remove(vertex3);
    }
    if (newEdge != NULL) {
        if (edgePointer1->orient == OrientLeft) {
            edge1->changeEndPoint(FirstPoint, newVertex, meshManager, flowManager);
        } else {
            edge1->changeEndPoint(SecondPoint, newVertex, meshManager, flowManager);
        }
        if (newEdge != NULL && polygon3 != NULL)
            edge1->detectAgent.handoverVertices(newEdge);
        edge1->detectAgent.updateVertexProjections(meshManager);
    }
    // -------------------------------------------------------------------------
    // handle degenerate polygons
    if (polygon1 != NULL && polygon1->edgePointers.size() == 1) {
        handlePointPolygon(polygonManager, polygon1, true);
        polygon1->handoverTracers(polygon3, 1.0);
        polygon1 = NULL;
    }
    if (polygon3 != NULL && polygon3->edgePointers.size() == 1) {
        handlePointPolygon(polygonManager, polygon3);
        polygon3 = NULL;
    }
    if (polygon1 != NULL && polygon1->edgePointers.size() == 2) {
        handleLinePolygon(polygonManager, polygon1, true);
        if (polygon3 != NULL)
            polygon1->handoverTracers(polygon3, 1.0);
        polygon1 = NULL;
    }
    if (polygon3 != NULL && polygon3->edgePointers.size() == 2) {
        handleLinePolygon(polygonManager, polygon3);
        polygon3 = NULL;
    }
    if (polygon1 != NULL) {
        if (handleSlimPolygon(meshManager, flowManager,
                              polygonManager, polygon1, true)) {
            if (polygon3 != NULL)
                polygon1->handoverTracers(polygon3, 1.0);
            polygon1 = NULL;
        }
    }
    if (polygon3 != NULL) {
        if (handleSlimPolygon(meshManager, flowManager,
                              polygonManager, polygon3))
            polygon3 = NULL;
    }
    if (polygon4 != NULL)
        if (polygon4->edgePointers.size() == 2)
            handleLinePolygon(polygonManager, polygon4);
    // -------------------------------------------------------------------------
    CommonTasks::doTask(CommonTasks::UpdateAngle);
    // -------------------------------------------------------------------------
    // hand over tracer mass
    if (polygon1 != NULL && polygon3 != NULL) {
        // BUG: The old area may be overwritten by later calling of calcArea.
        polygon1->calcArea();
        polygon3->calcArea();
        double percent = polygon3->getArea()/(polygon1->getArea()+polygon3->getArea());
        polygon1->handoverTracers(polygon3, percent);
    }
    // -------------------------------------------------------------------------
    // detect the new vertex for approaching
    linkedEdge = newVertex->linkedEdges.front();
    for (i = 0; i < newVertex->linkedEdges.size(); ++i) {
        Edge *edge = linkedEdge->edge;
        if (edge->getEndPoint(FirstPoint) == newVertex)
            orient = OrientLeft;
        else if (edge->getEndPoint(SecondPoint) == newVertex)
            orient = OrientRight;
#ifndef DEBUG
        cout << "Detecting polygon ";
        cout << edge->getPolygon(orient)->getID() << endl;
#endif
        detectPolygon(meshManager, flowManager, polygonManager,
                      edge->getPolygon(orient));
        linkedEdge = linkedEdge->next;
    }
    if (mode > 2) {
#ifndef DEBUG
        cout << "New vertex " << newVertex->getID() << " is linked to ";
        cout << newVertex->linkedEdges.size() << " edges." << endl;
        REPORT_DEBUG
#endif
    }
    // -------------------------------------------------------------------------
    projection = newVertex->detectAgent.getProjection(crossedEdge);
    if (projection != NULL) {
        if (newVertex->detectAgent.getActiveProjection() == NULL)
            ApproachingVertices::recordVertex(newVertex);
        projection->setApproach(true);
        newVertex->tags.set(MayCrossEdge);
    }
}

bool handleApproachEvents(MeshManager &meshManager,
                          const FlowManager &flowManager,
                          PolygonManager &polygonManager)
{
    bool isSplit = false;
    Vertex *vertex3, *vertex4;
    Edge *edge1;
    EdgePointer *edgePointer1, *edgePointer2;
    Polygon *polygon1;
    int i;
    EdgePointer *linkedEdge;
    Projection *projection;

#ifdef DEBUG
    assert(CommonTasks::getTaskNumber(CommonTasks::UpdateAngle) == 0);
#endif

    while (!ApproachingVertices::isEmpty()) {
        vertex3 = ApproachingVertices::vertices.front();
//        if (TimeManager::getSteps() == 375 && (vertex3->getID() == 620874 ||
//                                               vertex3->getID() == 612384))
//            REPORT_DEBUG;
        // ---------------------------------------------------------------------
        // if the vertex3 is a test point, split its edge
        // TODO: Will there be test points here?
        if (vertex3->getID() == -1) {
            if (splitEdge(meshManager, flowManager, polygonManager,
                          vertex3->getHostEdge()))
                vertex3 = polygonManager.vertices.back();
            else {
                static_cast<TestPoint *>(vertex3)->reset(meshManager);
                continue;
            }
        }
        // ---------------------------------------------------------------------
        // collect information
        projection = vertex3->detectAgent.getActiveProjection();
        edge1 = projection->getEdge();
        // ---------------------------------------------------------------------
        // check if there is another approaching vertex whose projection
        // distance is smaller than vertex3
        bool hasAnotherVertex = false;
        list<Vertex *>::const_iterator it = edge1->detectAgent.vertices.begin();
        for (; it != edge1->detectAgent.vertices.end(); ++it) {
            vertex4 = *it;
            if (vertex4->getID() == -1 || vertex3 == vertex4)
                continue;
            Projection *projection1 = vertex4->detectAgent.getProjection(edge1);
            if (projection1->getOrient() != projection->getOrient())
                continue;
            if (projection1->getDistance(NewTimeLevel) <
                projection->getDistance(NewTimeLevel)) {
                // check if the linked edge of vertex4 will block vertex3 or not
                linkedEdge = vertex4->linkedEdges.front();
                for (i = 0; i < vertex4->linkedEdges.size(); ++i) {
                    Projection *projection2 = vertex3->detectAgent.
                    getProjection(linkedEdge->edge);
                    if (projection2 != NULL &&
                        projection2->getDistance(NewTimeLevel) <
                        projection->getDistance(NewTimeLevel)) {
                        if (projection1->isApproaching()) {
                            ApproachingVertices::jumpVertex(vertex3, vertex4);
                        } else {
                            projection->setApproach(false);
                            if (vertex3->detectAgent.getActiveProjection() == NULL)
                                ApproachingVertices::removeVertex(vertex3);
                        }
                        hasAnotherVertex = true;
                        break;
                    }
                    linkedEdge = linkedEdge->next;
                }
                if (!hasAnotherVertex) {
                    hasAnotherVertex = true;
                    if (projection1->isApproaching()) {
                        ApproachingVertices::jumpVertex(vertex3, vertex4);
                    } else {
                        projection->setApproach(false);
                        if (vertex3->detectAgent.getActiveProjection() == NULL)
                            ApproachingVertices::removeVertex(vertex3);
                    }
                    break;
                }
            }
        }
        if (hasAnotherVertex)
            continue;
        // ---------------------------------------------------------------------
        edgePointer2 = NULL;
        edgePointer1 = edge1->getEdgePointer(projection->getOrient());
        polygon1 = edge1->getPolygon(projection->getOrient());
        linkedEdge = vertex3->linkedEdges.front();
        for (i = 0; i < vertex3->linkedEdges.size(); ++i) {
            if (linkedEdge->edge->getPolygon(OrientLeft) == polygon1 &&
                linkedEdge->edge->getEndPoint(SecondPoint) == vertex3) {
                edgePointer2 = linkedEdge->edge->getEdgePointer(OrientLeft);
                break;
            } else if (linkedEdge->edge->getPolygon(OrientRight) == polygon1 &&
                       linkedEdge->edge->getEndPoint(FirstPoint) == vertex3) {
                edgePointer2 = linkedEdge->edge->getEdgePointer(OrientRight);
                break;
            }
            linkedEdge = linkedEdge->next;
        }
#ifdef DEBUG
        if (edgePointer2 == NULL) {
            // Note: There are possibilities that vertex3 is not a vertex of
            //       polygon1, so we should ignore vertex3.
            bool isReallyCross = false;
            linkedEdge = vertex3->linkedEdges.front();
            for (i = 0; i < vertex3->linkedEdges.size(); ++i) {
                if (linkedEdge->edge->getPolygon(OrientLeft) == polygon1 ||
                    linkedEdge->edge->getPolygon(OrientRight) == polygon1) {
                    isReallyCross = true;
                    break;
                }
                linkedEdge = linkedEdge->next;
            }
            if (!isReallyCross) {
                ApproachDetector::AgentPair::unpair(vertex3, edge1);
                if (vertex3->detectAgent.getActiveProjection() == NULL)
                    ApproachingVertices::removeVertex(vertex3);
                continue;
            }
            Message message;
            message << "Edge-crossing event has occurred!\n";
            message << "polygon1: " << polygon1->getID() << "\n";
            message << "edge1: " << edge1->getID() << "\n";
            message << "vertex3: " << vertex3->getID() << "\n";
            REPORT_ERROR(message.str());
        }
#endif
        // ---------------------------------------------------------------------
#ifndef DEBUG
        DebugTools::assert_consistent_projection(projection);
        assert(edgePointer2->getEndPoint(SecondPoint) == vertex3);
#endif
        isSplit = true;
        // ---------------------------------------------------------------------
        int mode = ApproachDetector::chooseMode(edgePointer1, vertex3,
                                                projection);
        if (mode == -1) {
            vertex3->detectAgent.getProjection(edge1)->setApproach(false);
            if (vertex3->detectAgent.getActiveProjection() == NULL)
                ApproachingVertices::removeVertex(vertex3);
            continue;
        }
        splitPolygon(meshManager, flowManager, polygonManager, polygon1,
                     edgePointer1, edgePointer2, vertex3, mode);
    }
    return isSplit;
}

inline bool isIntrudingVertex(EdgePointer *markEdgePointer)
{
    static const double a0 = 250.0/Rad2Deg;
    EdgePointer *edgePointer1, *edgePointer2;
    Edge *edge1, *edge2, *edge3, *edge4;
    Vertex *vertex1, *vertex2, *vertex3, *vertex4, *vertex5;
    OrientStatus orient1, orient2, orient3, orient4;
    Projection *projection;

    edge1 = markEdgePointer->prev->edge;
    edge2 = markEdgePointer->edge;
    vertex1 = markEdgePointer->prev->getEndPoint(FirstPoint);
    vertex2 = markEdgePointer->getEndPoint(FirstPoint);
    vertex3 = markEdgePointer->getEndPoint(SecondPoint);
    orient1 = markEdgePointer->prev->orient;
    orient2 = markEdgePointer->orient;
    if (markEdgePointer->getAngle() > a0) {
        edgePointer1 = markEdgePointer->next;
        edgePointer2 = markEdgePointer->prev->prev;
        do {
            edge3 = edgePointer1->edge;
            edge4 = edgePointer2->edge;
            vertex4 = edgePointer1->getEndPoint(SecondPoint);
            vertex5 = edgePointer2->getEndPoint(FirstPoint);
            orient3 = edgePointer1->orient;
            orient4 = edgePointer2->orient;
            projection = vertex4->detectAgent.getProjection(edge1);
            if (projection != NULL && projection->getOrient() != orient1)
                return true;
            projection = vertex1->detectAgent.getProjection(edge3);
            if (projection != NULL && projection->getOrient() != orient3)
                return true;
            projection = vertex5->detectAgent.getProjection(edge2);
            if (projection != NULL && projection->getOrient() != orient2)
                return true;
            projection = vertex3->detectAgent.getProjection(edge4);
            if (projection != NULL && projection->getOrient() != orient4)
                return true;
            edgePointer1 = edgePointer1->next;
            edgePointer2 = edgePointer2->prev;
        } while (edgePointer1->getAngle() > PI &&
                 edgePointer2->getAngle() > PI &&
                 edgePointer1 != edgePointer2);
    }
    return false;
}

bool handleBentPolygons(MeshManager &meshManager,
                        const FlowManager &flowManager,
                        PolygonManager &polygonManager)
{
    bool isSplit = false;
    std::list<Projection>::const_iterator itPrj;
    std::list<Vertex *>::const_iterator itVtx;
    Vertex *vertex1;
    Edge *markEdge, *newEdge;
    EdgePointer *edgePointer;
    EdgePointer *markEdgePointer1, *markEdgePointer2;
    EdgePointer *newEdgePointer1, *newEdgePointer2;
    const Projection *projection;
    Vector vector1, vector2;
    double distance, angle1, angle2, angle3, angle4;
    bool isConnectOk1, isConnectOk2;
    int mode;
    Location loc;
    // -------------------------------------------------------------------------
    Polygon *newPolygon;
    Polygon *polygon = polygonManager.polygons.front();
    while (polygon != NULL) {
//        if (TimeManager::getSteps() >= 448 && polygon->getID() == 6503)
//            REPORT_DEBUG;
    rehandle_polygon:
        markEdgePointer1 = polygon->edgePointers.front();
        do {
            markEdgePointer2 = NULL; distance = 1.0e33;
            // -----------------------------------------------------------------
            // find out the vertex that is penetrating into polygon
            if (isIntrudingVertex(markEdgePointer1)) {
                vertex1 = markEdgePointer1->getEndPoint(FirstPoint);
                for (itPrj = vertex1->detectAgent.getProjections().begin();
                     itPrj != vertex1->detectAgent.getProjections().end(); ++itPrj) {
                    projection = &*itPrj;
                    // Note: Find the most close edge to split the polygon
                    if (distance < projection->getDistance(NewTimeLevel))
                        continue;
                    markEdge = projection->getEdge();
                    if (markEdge != markEdgePointer1->prev->prev->edge &&
                        markEdge != markEdgePointer1->next->edge) {
                        if (markEdge->getPolygon(OrientLeft) == polygon) {
                            markEdgePointer2 = markEdge->getEdgePointer(OrientLeft);
                            distance = projection->getDistance(NewTimeLevel);
                        } else if (markEdge->getPolygon(OrientRight) == polygon) {
                            markEdgePointer2 = markEdge->getEdgePointer(OrientRight);
                            distance = projection->getDistance(NewTimeLevel);
                        } else
                            continue;
                        if (projection->getOrient() != markEdgePointer2->orient) {
                            markEdgePointer2 = NULL; distance = 1.0e33;
                            continue;
                        }
                        ;
                        if (detectAddConnection(polygon, markEdgePointer1,
                                                markEdgePointer2, isConnectOk1,
                                                isConnectOk2, vector1,
                                                vector2) == Cross) {
                            markEdgePointer2 = NULL; distance = 1.0e33;
                        }
                    }
                }
            }
            // -----------------------------------------------------------------
            if (markEdgePointer2 != NULL) {
                // choose which vertex to connect
                if (isConnectOk1 && isConnectOk2) {
                    angle1 = EdgePointer::calcAngle
                    (markEdgePointer1->prev->getNormVector(), -vector1, *vertex1);
                    angle2 = EdgePointer::calcAngle
                    (vector1, markEdgePointer1->getNormVector(), *vertex1);
                    angle3 = EdgePointer::calcAngle
                    (markEdgePointer1->prev->getNormVector(), -vector2, *vertex1);
                    angle4 = EdgePointer::calcAngle
                    (vector2, markEdgePointer1->getNormVector(), *vertex1);
#ifndef DEBUG
                    assert(fabs(angle1+angle2-angle3-angle4) < EPS);
                    assert(fabs(angle1+angle2-markEdgePointer1->getAngle()) < EPS);
#endif
                    if (fmax(angle1, angle2) < fmax(angle3, angle4)) {
                        mode = 1;
                    } else {
                        mode = 2;
                    }
                } else {
                    if (isConnectOk1)
                        mode = 1;
                    if (isConnectOk2)
                        mode = 2;
                }
                // create new polygon
                polygonManager.polygons.append(&newPolygon);
                if (mode == 2)
                    markEdgePointer2 = markEdgePointer2->next;
                edgePointer = markEdgePointer1;
                while (edgePointer != markEdgePointer2) {
                    newPolygon->edgePointers.append(&newEdgePointer2);
                    newEdgePointer2->replace(edgePointer);
                    newEdgePointer2->edge->setPolygon(newEdgePointer2->orient,
                                                      newPolygon);
                    edgePointer = edgePointer->next;
                    polygon->edgePointers.remove(edgePointer->prev);
                }
                // create new edge
                polygonManager.edges.append(&newEdge);
                newEdge->linkEndPoint(FirstPoint, vertex1);
                newEdge->linkEndPoint(SecondPoint,
                                      markEdgePointer2->getEndPoint(FirstPoint));
                newEdge->calcNormVector();
                newEdge->calcLength();
                Vertex *testPoint = newEdge->getTestPoint();
                meshManager.checkLocation(testPoint->getCoordinate(), loc);
                testPoint->setLocation(loc);
                TTS::track(meshManager, flowManager, testPoint);
                newEdge->setPolygon(OrientLeft, polygon);
                newEdge->setPolygon(OrientRight, newPolygon);
                polygon->edgePointers.insert(&newEdgePointer1, markEdgePointer2);
                newPolygon->edgePointers.append(&newEdgePointer2);
                newPolygon->edgePointers.ring();
                newEdge->setEdgePointer(OrientLeft, newEdgePointer1);
                newEdge->setEdgePointer(OrientRight, newEdgePointer2);
                // update angles
                CommonTasks::doTask(CommonTasks::UpdateAngle);
                // hand over tracer mass
                polygon->calcArea();
                newPolygon->calcArea();
                double percent = newPolygon->getArea()/(polygon->getArea()+newPolygon->getArea());
                polygon->handoverTracers(newPolygon, percent);
                // detect the two polygons
                detectPolygon(meshManager, flowManager, polygonManager, polygon);
                detectPolygon(meshManager, flowManager, polygonManager, newPolygon);
#ifndef DEBUG
                cout << "Polygon " << polygon->getID() << " is split with new ";
                cout << "polygon " << newPolygon->getID() << endl;
#endif
                goto rehandle_polygon;
            } else
                markEdgePointer1 = markEdgePointer1->next;
        } while (markEdgePointer1 != polygon->edgePointers.back()->next);
        polygon = polygon->next;
    }
    return isSplit;
}

bool CurvatureGuard::splitPolygons(MeshManager &meshManager,
                                   const FlowManager &flowManager,
                                   PolygonManager &polygonManager)
{
    bool isSplit = false;
    if (handleBentPolygons(meshManager, flowManager, polygonManager))
        isSplit = true;
    if (handleApproachEvents(meshManager, flowManager, polygonManager))
        isSplit = true;
    return isSplit;
}

#endif