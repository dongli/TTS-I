#include "ApproachDetector.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "SpecialPolygons.h"
#include "CurvatureGuard.h"
#include "PotentialCrossDetector.h"
#include "TTS.h"
#include "CommonTasks.h"
#ifdef DEBUG_TTS
#include "DebugTools.h"
#endif

using namespace ApproachDetector;
using namespace SpecialPolygons;
using namespace CurvatureGuard;
using namespace PotentialCrossDetector;

double ApproachDetector::approachTrendThreshold(double distance)
{
    static const double D0 = 0.001/Rad2Deg*Sphere::radius;
    static const double D1 = 0.5/Rad2Deg*Sphere::radius;
    static const double P0 = 0.4;
    static const double P1 = 0.8;
    static const double dD = D1-D0;
    static const double dP = P1-P0;
    
    if (distance > D0 && distance < D1) {
        double t = (distance-D0)/dD;
        return dP*(4.0-3.0*t)*pow(t, 3.0)+P0;
    } else if (distance <= D0) {
        return P0;
    } else {
        return P1;
    }
}

bool ApproachDetector::isNeedCheck(double distance)
{
    static const double distanceThreshold = 5.0/Rad2Deg*Sphere::radius;
    if (distance < distanceThreshold)
        return true;
    else
        return false;
}

bool ApproachDetector::isApproaching(Projection *projection)
{
    static const double smallDistance = 0.05/Rad2Deg*Sphere::radius;
    double oldDistance = projection->getDistance(OldTimeLevel);
    double newDistance = projection->getDistance(NewTimeLevel);
    if (oldDistance != UndefinedDistance && newDistance != UndefinedDistance) {
        projection->calcChangeRate();
        double rate0 = approachTrendThreshold(oldDistance);
        if (projection->getChangeRate() > rate0 || newDistance < smallDistance)
            return true;
    } else if (newDistance != UndefinedDistance && newDistance < smallDistance) {
        return false;
    }
    return false;
}

inline void detectPoint(MeshManager &meshManager, const FlowManager &flowManager,
                        PolygonManager &polygonManager, Vertex *point,
                        EdgePointer *edgePointer1, EdgePointer *edgePointer2,
                        bool &isPointCrossEdge)
{
    static Projection p;
    bool isTestPointHasOldProjection = true; // to check crossing of test point
    Edge *edge = edgePointer1->edge;
    Projection *projection = point->detectAgent.getProjection(edge);
    if (projection == NULL && point->getID() == -1)
        isTestPointHasOldProjection = false;
    isPointCrossEdge = false;
    if (projection == NULL) {
        // ---------------------------------------------------------------------
        // Scenario 1:
        //   Vertex3 moves into the detecting range of edge1, but is
        //   not paired with it.
        projection = &p;
        projection->reinit();
        if (projection->project(point, edge, NewTimeLevel)) {
            // -----------------------------------------------------------------
            // Scenario 1-1:
            //   Vertex3 has projection on edge1 at new time step,
            //   but it is far away from edge1.
            if (!isNeedCheck(projection->getDistance(NewTimeLevel)))
                return;
            // -----------------------------------------------------------------
            // Note: Here we capture the crossing test point, which has not been
            //       detected.
            if (!isTestPointHasOldProjection &&
                detectTestPoint(edgePointer1, edgePointer2) == Cross) {
                static_cast<TestPoint *>(point)->reset(meshManager);
                if (projection->project(point, edge, NewTimeLevel)) {
                    projection->project(point, edge, OldTimeLevel);
                    AgentPair::pair(point, edge, projection);
                }
            } else {
                // -------------------------------------------------------------
                // Scenario 1-2:
                //   Vertex3 has projection on edge1 at new time step.
                projection->project(point, edge, OldTimeLevel);
                projection->checkApproaching();
                if (projection->isApproaching() &&
                    point->detectAgent.getActiveProjection() == NULL) {
                    ApproachingVertices::recordVertex(point);
                }
                AgentPair::pair(point, edge, projection);
            }
        }
    } else {
        // ---------------------------------------------------------------------
        // Scenario 2:
        //   Vertex3 is paired with edge1, and is still in the
        //   detecting range of edge1.
        // ---------------------------------------------------------------------
        // Scenario 2-1:
        //   The projection of vertex3 on edge1 has already been
        //   calculated by other procedures.
        if (projection->isCalculated())
            return;
        ProjectionStatus status = projection->project(NewTimeLevel);
        if (status == HasProjection) {
            // -----------------------------------------------------------------
            // Note: Here we capture the crossing test point, which has not been
            //       detected.
            if (!isTestPointHasOldProjection &&
                detectTestPoint(edgePointer1, edgePointer2) == Cross) {
                static_cast<TestPoint *>(point)->reset(meshManager);
            } else {
                // -------------------------------------------------------------
                // Scenario 2-1:
                //   Vertex3 has projection on edge1 at new time step.
                projection->checkApproaching();
                if (projection->isApproaching())
                    ApproachingVertices::recordVertex(point);
            }
        } else if (status == HasNoProjection) {
            // -----------------------------------------------------------------
            // Scenario 2-2:
            //   Vertex3 has no projection on edge1 at new time step.
            AgentPair::unpair(point, edge);
        } else if (status == CrossEdge) {
            if (point->getID() == -1) {
                // Note: When the test point cross the paired edge, we can
                //       only to reset it to avoid potential problems.
                static_cast<TestPoint *>(point)->reset(meshManager);
            } else {
                // TEST: When the point crosses the edge, we should remedy
                //       this problem insteal of throughing an error.
                isPointCrossEdge = true;
            }
        }
    }
}

bool ApproachDetector::checkApproachValid(MeshManager &meshManager,
                                          const FlowManager &flowManager,
                                          PolygonManager &polygonManager,
                                          EdgePointer *edgePointer1,
                                          EdgePointer *edgePointer2,
                                          Vertex *vertex3)
{
    Projection *projection;
    // -------------------------------------------------------------------------
    // collect information
    projection = vertex3->detectAgent.getProjection(edgePointer1->edge);
    if (projection == NULL)
        return false;
    // -------------------------------------------------------------------------
    if (projection->isApproaching()) {
        if (chooseMode(edgePointer1, vertex3, projection) == -1)
            goto return_invalid_approach;
    } else
        return false;
    // -------------------------------------------------------------------------
    if (vertex3->getID() == -1) {
        if (edgePointer2->prev == edgePointer1 ||
            edgePointer2->next == edgePointer1)
            goto return_invalid_approach;
        if (edgePointer2->getEndPoint(FirstPoint)->detectAgent.
            getActiveProjection() == NULL &&
            edgePointer2->getEndPoint(SecondPoint)->detectAgent.
            getActiveProjection() == NULL) {
            if (!splitEdge(meshManager, flowManager, polygonManager,
                           vertex3->getHostEdge()))
                goto return_reset_test_point;
            else
                REPORT_DEBUG;
        } else
            goto return_invalid_approach;
    }
    return true;
return_reset_test_point:
    if (vertex3->getID() == -1) {
        static_cast<TestPoint *>(vertex3)->reset(meshManager);
    } else {
        edgePointer1->edge->getTestPoint()->reset(meshManager);
        return false;
    }
return_invalid_approach:
    projection->setApproach(false);
    if (vertex3->detectAgent.getActiveProjection() == NULL)
        ApproachingVertices::removeVertex(vertex3);
    return false;
}

//#define DIAG_EDGE_LENGTH

void ApproachDetector::detectPolygon(MeshManager &meshManager,
                                     const FlowManager &flowManager,
                                     PolygonManager &polygonManager,
                                     Polygon *polygon)
{
//    if (TimeManager::getSteps() >= 304 && (polygon->getID() == 12511)) {
//        DebugTools::watch(polygon);
//        polygon->dump("polygon");
//        REPORT_DEBUG;
//    }
    // -------------------------------------------------------------------------
    static std::list<Vertex *> crossVertices;
    Vertex *vertex3;
    EdgePointer *edgePointer3 = NULL, *edgePointer4 = NULL;
    static bool handleCrossVertices = false;
    // -------------------------------------------------------------------------
    if (polygon->edgePointers.size() == 2) {
        handleLinePolygon(polygonManager, polygon);
        // TODO: Hand over the tracer mass.
        return;
    }
    // -------------------------------------------------------------------------
    bool isPointCrossEdge;
    EdgePointer *edgePointer1 = polygon->edgePointers.front();
    for (int i = 0; i < polygon->edgePointers.size(); ++i) {
        EdgePointer *edgePointer2 = edgePointer1->next;
        EdgePointer *nextEdgePointer2;
        while (edgePointer2 != edgePointer1) {
            nextEdgePointer2 = edgePointer2->next;
            TestPoint *testPoint;
            // -----------------------------------------------------------------
            if (edgePointer2 != edgePointer1->prev) {
                vertex3 = edgePointer2->getEndPoint(SecondPoint);
                if (handleCrossVertices &&
                    find(crossVertices.begin(), crossVertices.end(), vertex3) !=
                    crossVertices.end()) {
                    crossVertices.remove(vertex3);
                    if (edgePointer3 != NULL)
                        splitPolygon(meshManager, flowManager, polygonManager,
                                     polygon, edgePointer3, edgePointer4, vertex3, 5);
                    else
                        splitPolygon(meshManager, flowManager, polygonManager,
                                     polygon, edgePointer1, edgePointer2, vertex3, 5);
                    return;
                } else
                    detectPoint(meshManager, flowManager, polygonManager, vertex3,
                                edgePointer1, edgePointer2, isPointCrossEdge);
                if (isPointCrossEdge) {
                    if (edgePointer3 == NULL) {
                        edgePointer3 = edgePointer1;
                        edgePointer4 = edgePointer2;
                    }
                    crossVertices.push_back(vertex3);
                }
                checkApproachValid(meshManager, flowManager, polygonManager,
                                   edgePointer1, edgePointer2, vertex3);
            }
            // -----------------------------------------------------------------
            testPoint = edgePointer2->edge->getTestPoint();
            detectPoint(meshManager, flowManager, polygonManager, testPoint,
                        edgePointer1, edgePointer2, isPointCrossEdge);
            checkApproachValid(meshManager, flowManager, polygonManager,
                               edgePointer1, edgePointer2, testPoint);
            // -----------------------------------------------------------------
            edgePointer2 = nextEdgePointer2;
        }
        edgePointer1 = edgePointer1->next;
    }
    if (handleCrossVertices)
        return;
    if (crossVertices.size() != 0) {
#ifdef DEBUG
        cout << "Crossing vertex number: " << crossVertices.size() << endl;
        std::list<Vertex *>::const_iterator it;
        for (it = crossVertices.begin(); it != crossVertices.end(); ++it) {
            cout << "  * " << (*it)->getID() << endl;
        }
#endif
        vertex3 = crossVertices.front();
        crossVertices.remove(vertex3);
        handleCrossVertices = true;
        splitPolygon(meshManager, flowManager, polygonManager,
                     polygon, edgePointer3, edgePointer4, vertex3, 5);
        handleCrossVertices = false;
    }
#ifdef DEBUG
    assert(crossVertices.size() == 0);
#endif
#ifdef DIAG_EDGE_LENGTH
    double minEdgeLength = 1.0e34, maxEdgeLength = -1.0e34;
    edgePointer1 = polygon->edgePointers.front();
    for (int i = 0; i < polygon->edgePointers.size(); ++i) {
        if (edgePointer1->edge->getLength() > maxEdgeLength)
            maxEdgeLength = edgePointer1->edge->getLength();
        if (edgePointer1->edge->getLength() < minEdgeLength)
            minEdgeLength = edgePointer1->edge->getLength();
        edgePointer1 = edgePointer1->next;
    }
    cout << "Min edge length: " << minEdgeLength*Rad2Deg/Sphere::radius << endl;
    cout << "Max edge length: " << maxEdgeLength*Rad2Deg/Sphere::radius << endl;
#endif
}

void ApproachDetector::detectPolygons(MeshManager &meshManager,
                                      const FlowManager &flowManager,
                                      PolygonManager &polygonManager)
{
    Polygon *polygon = polygonManager.polygons.front();
    Polygon *nextPolygon;
    while (polygon != NULL) {
        nextPolygon = polygon->next;
        detectPolygon(meshManager, flowManager, polygonManager, polygon);
        polygon = nextPolygon;
    }
}

void ApproachDetector::reset(PolygonManager &polygonManager)
{
    Vertex *vertex = polygonManager.vertices.front();
    for (int i = 0; i < polygonManager.vertices.size(); ++i) {
        vertex->detectAgent.expireProjection();
        // TODO: We reset the tags of vertices here. Should we do this somewhere
        //       else?
        vertex->tags.reset();
        vertex = vertex->next;
    }
    Edge *edge = polygonManager.edges.front();
    for (int i = 0; i < polygonManager.edges.size(); ++i) {
        edge->getTestPoint()->detectAgent.expireProjection();
        // TODO: We reset the tags of edges here. Should we do this somewhere
        //       else?
        edge->tags.reset();
        edge = edge->next;
    }
}

int ApproachDetector::chooseMode(EdgePointer *edgePointer1,
                                 Vertex *vertex3, Projection *projection)
{
    // TODO: Avoid use the absolute distance.
    static const double smallDistance1 = 0.06/Rad2Deg*Sphere::radius;
    static const double smallDistance2 = 0.06/Rad2Deg*Sphere::radius;
    const Coordinate &x1 = edgePointer1->getEndPoint(FirstPoint)->getCoordinate();
    const Coordinate &x2 = edgePointer1->getEndPoint(SecondPoint)->getCoordinate();
    double d1 = Sphere::calcDistance(x1, vertex3->getCoordinate());
    double d2 = Sphere::calcDistance(x2, vertex3->getCoordinate());
    double d3;
    if (d1 <= d2)
        d3 = Sphere::calcDistance(x1, projection->getCoordinate(NewTimeLevel));
    else if (d2 <= d1)
        d3 = Sphere::calcDistance(x2, projection->getCoordinate(NewTimeLevel));
    double d4 = projection->getDistance(NewTimeLevel);
    edgePointer1->edge->calcLength();
    // branch-1: avoid wild changes
    if (d4/edgePointer1->edge->getLength() > 0.1)
        return -1;
    // branch-2
    if (d1 <= d2) {
        if (d3 <= smallDistance2) {
            if (d1 <= smallDistance1 && 1.0-d1/d2 > 0.5)
                return 1;
            else {
                if (projection->getChangeRate() <= 0.45 &&
                    !vertex3->tags.isSet(MayCrossEdge))
                    return -1;
                else {
                    if (d4 < smallDistance1 ||
                        vertex3->tags.isSet(MayCrossEdge))
                        return 4;
                    else
                        return -1;
                }
            }
        } else {
            return 4;
        }
    } else {
        if (d3 <= smallDistance2) {
            if (d2 <= smallDistance1 && 1.0-d2/d1 > 0.5)
                return 2;
            else {
                if (projection->getChangeRate() <= 0.45 &&
                    !vertex3->tags.isSet(MayCrossEdge))
                    return -1;
                else {
                    if (d4 < smallDistance1 ||
                        vertex3->tags.isSet(MayCrossEdge))
                        return 4;
                    else
                        return -1;
                }
            }
        } else {
            return 4;
        }
    }
}