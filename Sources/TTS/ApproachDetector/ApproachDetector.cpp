#include "ApproachDetector.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "SpecialPolygons.h"
#ifdef DEBUG
#include "TimeManager.h"
#include "DebugTools.h"
#endif

using namespace SpecialPolygons;

double ApproachDetector::approachTrendThreshold(double distance)
{
    static const double D0 = 0.005/Rad2Deg*Sphere::radius;
    static const double D1 = 0.1/Rad2Deg*Sphere::radius;
    static const double P0 = 0.3;
    static const double P1 = 0.5;
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
        double ratio = (oldDistance-newDistance)/oldDistance;
        double ratio0 = approachTrendThreshold(oldDistance);
        if (ratio > ratio0 || newDistance < smallDistance)
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
    Projection p;
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
            if (projection->isCalculated())
                return NULL;
            if (projection->project(NewTimeLevel)) {
                // -------------------------------------------------------------
                // Scenario 2-1:
                //   Vertex3 has projection on edge1 at new time step.
                projection->checkApproaching();
                if (projection->isApproaching()) {
                    ApproachingVertices::recordVertex(point);
                    isApproaching = true;
                }
            } else
                // -------------------------------------------------------------
                // Scenario 2-2:
                //   Vertex3 has no projection on edge1 at new time step.
                AgentPair::unpair(point, edge);
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
//    if (TimeManager::getSteps() >= 73 && polygon->getID() == 3296) {
//        DebugTools::watch_polygon(polygon);
//        polygon->dump("polygon");
//        REPORT_DEBUG;
//    }
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
        for (int j = 0; j < polygon->edgePointers.size()-2; ++j) {
            Projection *projection; Vertex *vertex3;
            // -----------------------------------------------------------------
            vertex3 = edgePointer2->getEndPoint(SecondPoint);
            if (vertex3 == vertex1) {
                handleEnclosedPolygons(polygonManager, polygon,
                                       edgePointer1, edgePointer2);
                goto next_edge;
            } else if (vertex3 == vertex2) {
                handleEnclosedPolygons(polygonManager, polygon,
                                       edgePointer1->next, edgePointer2);
                goto next_edge;
            }
            projection = detectPoint(edge1, I1, I2, J1, J2, vertex3);
            // -----------------------------------------------------------------
//            vertex3 = edgePointer2->edge->getTestPoint()
//            projection = detectPoint(edge1, I1, I2, J1, J2, vertex3);
//            if (projection != NULL) {
//                
//            }
            edgePointer2 = edgePointer2->next;
        }
    next_edge: edgePointer1 = edgePointer1->next;
    }
}

void ApproachDetector::detect(MeshManager &meshManager,
                              const FlowManager &flowManager,
                              PolygonManager &polygonManager)
{
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
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
    static const double smallDistance = 0.05/Rad2Deg*Sphere::radius;
    double d1 = Sphere::calcDistance(vertex1->getCoordinate(),
                                     vertex3->getCoordinate());
    double d2 = Sphere::calcDistance(vertex2->getCoordinate(),
                                     vertex3->getCoordinate());
    if (d1 <= d2 && d1 < smallDistance)
        return 1;
    else if (d2 < d1 && d2 < smallDistance)
        return 2;
    else if (projection->getDistance(NewTimeLevel) > smallDistance)
        return 3;
    else
        return 4;
}