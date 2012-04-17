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
    Polygon *polygon2, *polygon3;
    EdgePointer *edgePointer3, *linkedEdge;
    Projection *projection;
    Location loc;
    OrientStatus orient;
    bool isVertexCross = false;
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
            isVertexCross = true;
            break;
        default:
            REPORT_ERROR("Unknown mode!");
    }
    // -------------------------------------------------------------------------
    // check if the split operation is ok
    if (mode != 5 &&
        detectReplaceVertex(edgePointer1, vertex3, testVertex) != NoCross) {
        if (mode == 1 || mode == 2) {
            projection->tags.unset(Approaching);
            projection->tags.set(CannotSplitPolygon);
            if (vertex3->detectAgent.getActiveProjection() == NULL)
                ApproachingVertices::removeVertex(vertex3);
            return;
        }
        // when mode is 1/2/4, vertex3 will be eliminated, and this
        // may cause edge-crossing. If this happens, shift to mode 5
        // by making testVertex be vertex3.
        // TODO: Whether we should split the polygon at this condition?
        testVertex = vertex3;
        mode = 5;
    }
    if (mode > 3) {
        if (detectInsertVertexOnEdge(meshManager, flowManager, polygonManager,
                                     edge1, testVertex, vertex3,
                                     &crossedEdge) != NoCross) {
            if (!isVertexCross && mode == 5) {
                projection->tags.unset(Approaching);
                projection->tags.set(CannotSplitPolygon);
                if (vertex3->detectAgent.getActiveProjection() == NULL)
                    ApproachingVertices::removeVertex(vertex3);
                return;
            }
            if (mode == 5) {
                testVertex = &vertex;
                mode = 4;
            }
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
        polygon1->handoverTracers(polygon3, 1.0);
        handlePointPolygon(polygonManager, polygon1, true);
        polygon1 = NULL;
    }
    if (polygon3 != NULL && polygon3->edgePointers.size() == 1) {
        handlePointPolygon(polygonManager, polygon3);
        polygon3 = NULL;
    }
    if (polygon1 != NULL && polygon1->edgePointers.size() == 2) {
        if (polygon3 != NULL) {
            polygon1->handoverTracers(polygon3, 1.0);
            handleLinePolygon(polygonManager, polygon1, newVertex, true);
        } else {
            handleLinePolygon(polygonManager, polygon1, newVertex);
        }
        polygon1 = NULL;
    }
    if (polygon3 != NULL && polygon3->edgePointers.size() == 2) {
        handleLinePolygon(polygonManager, polygon3, newVertex);
        polygon3 = NULL;
    }
    // -------------------------------------------------------------------------
    CommonTasks::doTask(CommonTasks::UpdateAngle);
    // -------------------------------------------------------------------------
    // hand over tracer mass
    // TODO: remember to handle over the enclosed polygon
    if (polygon1 != NULL && polygon3 != NULL) {
        // BUG: The old area may be overwritten by later calling of calcArea.
        polygon1->calcArea();
        polygon3->calcArea();
        double percent = polygon3->getArea()/(polygon1->getArea()+polygon3->getArea());
        polygon1->handoverTracers(polygon3, percent);
    }
#ifdef DEBUG
    DebugTools::assert_polygon_mass_constant(polygonManager);
#endif
    // -------------------------------------------------------------------------
    // detect the new vertex for approaching
    if (newVertex != NULL && newVertex->endTag != ListElement<Vertex>::Null) {
        newVertex->linkedEdges.startLoop(linkedEdge);
        do {
            Edge *edge = linkedEdge->edge;
            if (edge->getEndPoint(FirstPoint) == newVertex)
                orient = OrientLeft;
            else if (edge->getEndPoint(SecondPoint) == newVertex)
                orient = OrientRight;
            else
                REPORT_ERROR("New vertex has been removed!");
            detectPolygon(meshManager, flowManager, polygonManager,
                          edge->getPolygon(orient));
            linkedEdge = newVertex->linkedEdges.getNextElem();
        } while (!newVertex->linkedEdges.isLoopEnd(linkedEdge));
    }
    // -------------------------------------------------------------------------
    projection = newVertex->detectAgent.getProjection(crossedEdge);
    if (projection != NULL) {
        if (newVertex->detectAgent.getActiveProjection() == NULL)
            ApproachingVertices::recordVertex(newVertex);
        projection->tags.set(Approaching);
        newVertex->tags.set(MayCrossEdge);
    }
}

bool handleApproachEvents(MeshManager &meshManager,
                          const FlowManager &flowManager,
                          PolygonManager &polygonManager)
{
    bool isSplit = false;
    Vertex *vertex3;
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
//        if (TimeManager::getSteps() >= 686 && (vertex3->getID() == 1743113))
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
        edgePointer1 = edge1->getEdgePointer(projection->getOrient());
        edgePointer2 = NULL;
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
        // check if there is another edge that blocks the approaching path of
        // vertex3 and will be crossed if we handle this approaching event
        bool hasAnotherEdge = false;
        list<Projection>::iterator itPrj;
        for (itPrj = vertex3->detectAgent.getProjections().begin();
             itPrj != vertex3->detectAgent.getProjections().end(); ++itPrj) {
            Edge *edge2 = (*itPrj).getEdge();
            if (edge1 == edge2) continue;
            Vertex *vertex1, *vertex2;
            vertex1 = edge2->getEndPoint(FirstPoint);
            vertex2 = edge2->getEndPoint(SecondPoint);
            Projection *projection1, *projection2;
            projection1 = vertex1->detectAgent.getProjection(edge1);
            projection2 = vertex2->detectAgent.getProjection(edge1);
            if ((projection1 != NULL &&
                 projection1->getOrient() == projection->getOrient()) ||
                (projection2 != NULL &&
                 projection2->getOrient() == projection->getOrient()))
                if (Sphere::isIntersect(vertex1->getCoordinate(),
                                        vertex2->getCoordinate(),
                                        vertex3->getCoordinate(),
                                        projection->getCoordinate())) {
                    cout << "[Debug]: Branch 3: " << vertex3->getID() << endl;
                    assert((*itPrj).getDistance() < projection->getDistance());
                    if ((*itPrj).tags.isSet(CannotSplitPolygon)) {
                        projection->tags.unset(Approaching);
                        if (vertex3->detectAgent.getActiveProjection() == NULL)
                            ApproachingVertices::removeVertex(vertex3);
                    } else {
                        (*itPrj).tags.set(Approaching);
                        vertex3->tags.set(MayCrossEdge);
                    }
                    hasAnotherEdge = true;
                    break;
                }
        }
        if (hasAnotherEdge)
            continue;
        // ---------------------------------------------------------------------
#ifdef DEBUG
        DebugTools::assert_consistent_projection(projection);
        assert(edgePointer2->getEndPoint(SecondPoint) == vertex3);
#endif
        isSplit = true;
        // ---------------------------------------------------------------------
        int mode = ApproachDetector::chooseMode(edgePointer1, vertex3,
                                                projection);
        if (mode == -1) {
            vertex3->detectAgent.getProjection(edge1)->tags.unset(Approaching);
            if (vertex3->detectAgent.getActiveProjection() == NULL)
                ApproachingVertices::removeVertex(vertex3);
            continue;
        }
        splitPolygon(meshManager, flowManager, polygonManager, polygon1,
                     edgePointer1, edgePointer2, vertex3, mode);
    }
    return isSplit;
}

bool CurvatureGuard::splitPolygons(MeshManager &meshManager,
                                   const FlowManager &flowManager,
                                   PolygonManager &polygonManager)
{
    bool isSplit = false;
    if (handleApproachEvents(meshManager, flowManager, polygonManager))
        isSplit = true;
    return isSplit;
}

#endif
