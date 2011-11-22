#include "ApproachDetector.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "SpecialPolygons.h"
#include "CurvatureGuard.h"
#include "TTS.h"
#ifdef DEBUG
#include "TimeManager.h"
#include "DebugTools.h"
#endif

using namespace SpecialPolygons;
using namespace CurvatureGuard;

double ApproachDetector::approachTrendThreshold(double distance)
{
    static const double D0 = 0.001/Rad2Deg*Sphere::radius;
    static const double D1 = 2.0/Rad2Deg*Sphere::radius;
    static const double P0 = 0.3;
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
    static const double distanceThreshold = 0.2/Rad2Deg*Sphere::radius;
    if (distance < distanceThreshold)
        return true;
    else
        return false;
}

bool ApproachDetector::isApproaching(Projection *projection)
{
    static const double smallDistance = 0.02/Rad2Deg*Sphere::radius;
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

inline Projection *detectPoint(Edge *edge, int I1, int I2, int J1, int J2,
                               Vertex *point)
{
    bool isApproaching = false;
    static Projection p;
    Projection *projection = point->detectAgent.getProjection(edge);
    const Location &loc =point->getLocation();
    // -------------------------------------------------------------------------
    // use the PointCounter mesh as the first filter
    if ((loc.i[4] >= I1 && loc.i[4] <= I2) &&
        (loc.j[4] >= J1 && loc.j[4] <= J2)) {
        if (projection == NULL) {
            // -----------------------------------------------------------------
            // Scenario 1:
            //   Vertex3 moves into the detecting range of edge1, but is
            //   not paired with it.
            projection = &p;
            projection->reinit();
            if (projection->project(point, edge, NewTimeLevel)) {
                // -------------------------------------------------------------
                // Scenario 1-1:
                //   Vertex3 has projection on edge1 at new time step,
                //   but it is far away from edge1.
                if (!isNeedCheck(projection->getDistance(NewTimeLevel)))
                    return NULL;
                // -------------------------------------------------------------
                // Scenario 1-2:
                //   Vertex3 has projection on edge1 at new time step.
                projection->project(point, edge, OldTimeLevel);
                projection->checkApproaching();
                if (projection->isApproaching() &&
                    point->detectAgent.getActiveProjection() == NULL) {
                    ApproachingVertices::recordVertex(point);
                    isApproaching = true;
                }
                AgentPair::pair(point, edge, projection);
            }
        } else {
            // -----------------------------------------------------------------
            // Scenario 2:
            //   Vertex3 is paired with edge1, and is still in the
            //   detecting range of edge1.
            // -----------------------------------------------------------------
            // Scenario 2-1:
            //   The projection of vertex3 on edge1 has already been
            //   calculated by other procedures.
            if (projection->isCalculated()) {
                if (projection->isApproaching())
                    return projection;
                else
                    return NULL;
            }
            ProjectionStatus status = projection->project(NewTimeLevel);
            if (status == HasProjection) {
                // -------------------------------------------------------------
                // Scenario 2-1:
                //   Vertex3 has projection on edge1 at new time step.
                projection->checkApproaching();
                if (projection->isApproaching()) {
                    ApproachingVertices::recordVertex(point);
                    isApproaching = true;
                }
            } else if (status == HasNoProjection) {
                // -------------------------------------------------------------
                // Scenario 2-2:
                //   Vertex3 has no projection on edge1 at new time step.
                AgentPair::unpair(point, edge);
            } else if (status == CrossEdge)
                REPORT_DEBUG;
        }
    } else
        if (projection != NULL)
            // -----------------------------------------------------------------
            // Scenario 3:
            //   Vertex3 is paired with edge1, but moves out of the
            //   detecting range of edge1.
            AgentPair::unpair(point, edge);
    if (isApproaching)
        return projection;
    else
        return NULL;
}

void ApproachDetector::detect(MeshManager &meshManager,
                              const FlowManager &flowManager,
                              PolygonManager &polygonManager,
                              Polygon *polygon)
{
    if (TimeManager::getSteps() >= 10 && polygon->getID() == 1000) {
        DebugTools::watch(polygon);
        polygon->dump("polygon");
        REPORT_DEBUG;
    }
    // -------------------------------------------------------------------------
    // handle degenerate polygons
    if (polygon->edgePointers.size() == 2) {
        Polygon *polygon1 = polygon->edgePointers.front()->getPolygon(OrientRight);
        Polygon *polygon2 = polygon->edgePointers.back()->getPolygon(OrientRight);
        for (int i = 0; i < polygon->tracers.size(); ++i) {
            polygon1->tracers[i].addMass(polygon->tracers[i].getMass()*0.5);
            polygon2->tracers[i].addMass(polygon->tracers[i].getMass()*0.5);
        }
        handleLinePolygon(polygonManager, polygon);
        TTS::doTask(TTS::UpdateAngle);
        return;
    }
    // -------------------------------------------------------------------------
    EdgePointer *edgePointer1 = polygon->edgePointers.front();
    for (int i = 0; i < polygon->edgePointers.size(); ++i) {
        Edge *edge1 = edgePointer1->edge;
        Vertex *vertex1 = edgePointer1->getEndPoint(FirstPoint);
        Vertex *vertex2 = edgePointer1->getEndPoint(SecondPoint);
        int I1, I2, J1, J2;
        if (vertex1->getLocation().i[4] <= vertex2->getLocation().i[4]) {
            I1 = vertex1->getLocation().i[4]-1;
            I2 = vertex2->getLocation().i[4]+1;
        } else {
            I1 = vertex2->getLocation().i[4]-1;
            I2 = vertex1->getLocation().i[4]+1;
        }
        if (vertex1->getLocation().j[4] <= vertex2->getLocation().j[4]) {
            J1 = vertex1->getLocation().j[4]-1;
            J2 = vertex2->getLocation().j[4]+1;
        } else {
            J1 = vertex2->getLocation().j[4]-1;
            J2 = vertex1->getLocation().j[4]+1;
        }
        EdgePointer *edgePointer2 = edgePointer1->next;
        EdgePointer *nextEdgePointer;
        while (edgePointer2 != edgePointer1) {
            nextEdgePointer = edgePointer2->next;
            Projection *projection; Vertex *vertex3;
            // -----------------------------------------------------------------
            if (edgePointer2 != edgePointer1->prev) {
                vertex3 = edgePointer2->getEndPoint(SecondPoint);
                if (vertex3 == vertex1) {
                    handleEnclosedPolygons(polygonManager, polygon,
                                           edgePointer1, edgePointer2);
                    goto next_edge1;
                } else if (vertex3 == vertex2) {
                    handleEnclosedPolygons(polygonManager, polygon,
                                           edgePointer1->next, edgePointer2);
                    goto next_edge1;
                }
                projection = detectPoint(edge1, I1, I2, J1, J2, vertex3);
                if (projection != NULL &&
                    chooseMode(vertex1, vertex2, vertex3, projection) == -1) {
                    vertex3->detectAgent.getProjection(edge1)->setApproach(false);
                    if (vertex3->detectAgent.getActiveProjection() == NULL)
                        ApproachingVertices::removeVertex(vertex3);
                }
            }
            // -----------------------------------------------------------------
            vertex3 = edgePointer2->edge->getTestPoint();
            assert(vertex3->getHostEdge() == edgePointer2->edge);
            projection = detectPoint(edge1, I1, I2, J1, J2, vertex3);
            if (projection != NULL && projection->isApproaching()) {
                if ((edgePointer2->next->edge == edge1 ||
                     edgePointer2->prev->edge == edge1) &&
                    !(edgePointer1->getAngle(NewTimeLevel) < 20.0/Rad2Deg ||
                      PI2-edgePointer1->getAngle(NewTimeLevel) < 20.0/Rad2Deg) &&
                    projection->getChangeRate() <= 0.35) {
                    vertex3->detectAgent.getProjection(edge1)->setApproach(false);
                    if (vertex3->detectAgent.getActiveProjection() == NULL)
                        ApproachingVertices::removeVertex(vertex3);
                    goto next_edge2;
                }
                // Note: Only when both the end points are not approaching some
                //       edges, we split the edge.
                if (chooseMode(vertex1, vertex2, vertex3, projection) > 2 &&
                    edgePointer2->getEndPoint(FirstPoint)->detectAgent.
                    getActiveProjection() == NULL &&
                    edgePointer2->getEndPoint(SecondPoint)->detectAgent.
                    getActiveProjection() == NULL) {
                    if (splitEdge(meshManager, flowManager, polygonManager,
                                  vertex3->getHostEdge(), true)) {
                        vertex3 = polygonManager.vertices.back();
                    } else {
                        // Note: There are some occasions that the host edge of
                        //       vertex3 can not be split
                        AgentPair::unpair(vertex3, edge1);
                        if (vertex3->detectAgent.getActiveProjection() == NULL)
                            ApproachingVertices::removeVertex(vertex3);
                    }
                } else {
                    AgentPair::unpair(vertex3, edge1);
                    // Note: Make sure there is no other edges that vertex3 is
                    //       approaching.
                    if (vertex3->detectAgent.getActiveProjection() == NULL)
                        ApproachingVertices::removeVertex(vertex3);
                }
            }
            // -----------------------------------------------------------------
        next_edge2: edgePointer2 = nextEdgePointer;
        }
    next_edge1: edgePointer1 = edgePointer1->next;
    }
}

void ApproachDetector::detect(MeshManager &meshManager,
                              const FlowManager &flowManager,
                              PolygonManager &polygonManager)
{
    Polygon *endPolygon = polygonManager.polygons.back()->next;
    Polygon *polygon = polygonManager.polygons.front();
    while (polygon != endPolygon) {
        detect(meshManager, flowManager, polygonManager, polygon);
        polygon = polygon->next;
    }
}

void ApproachDetector::reset(PolygonManager &polygonManager)
{
    Vertex *vertex = polygonManager.vertices.front();
    for (int i = 0; i < polygonManager.vertices.size(); ++i) {
        vertex->detectAgent.expireProjection();
        vertex = vertex->next;
    }
    Edge *edge = polygonManager.edges.front();
    for (int i = 0; i < polygonManager.edges.size(); ++i) {
        edge->getTestPoint()->detectAgent.expireProjection();
        edge = edge->next;
    }
}

int ApproachDetector::chooseMode(Vertex *vertex1, Vertex *vertex2,
                                 Vertex *vertex3, Projection *projection)
{
    static const double smallDistance1 = 0.01/Rad2Deg*Sphere::radius;
    static const double smallDistance2 = 0.1/Rad2Deg*Sphere::radius;
    static const double smallDistance3 = 0.01/Rad2Deg*Sphere::radius;
    double d1 = Sphere::calcDistance(vertex1->getCoordinate(),
                                     vertex3->getCoordinate());
    double d2 = Sphere::calcDistance(vertex2->getCoordinate(),
                                     vertex3->getCoordinate());
    double d3;
    if (d1 <= d2)
        d3 = Sphere::calcDistance(vertex1->getCoordinate(),
                                  projection->getCoordinate(NewTimeLevel));
    else
        d3 = Sphere::calcDistance(vertex2->getCoordinate(),
                                  projection->getCoordinate(NewTimeLevel));
    double d4 = projection->getDistance(NewTimeLevel);
    if (d1 <= d2) {
        if (d3 <= smallDistance2) {
            if (projection->getChangeRate() <= 0.5)
                return -1;
            else {
                if (d1 <= smallDistance1)
                    return 1;
                else
                    if (d4 >= smallDistance3*1.5)
                        return 3;
                    else
                        return 4;
            }
        } else {
            if (d4 > smallDistance1)
                return 3;
            else
                return 4;
        }
    } else {
        if (d3 <= smallDistance2) {
            if (projection->getChangeRate() <= 0.5)
                return -1;
            else {
                if (d2 <= smallDistance1)
                    return 2;
                else
                    if (d4 >= smallDistance3*1.5)
                        return 3;
                    else
                        return 4;
            }
        } else {
            if (d4 > smallDistance1)
                return 3;
            else
                return 4;
        }
    }
}