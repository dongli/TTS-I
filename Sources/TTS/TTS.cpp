#include "TTS.h"
#include "Velocity.h"
#include "Point.h"
#include "TimeManager.h"
#include "Constants.h"
#include "Sphere.h"

TTS::TTS()
{
    A0 = 0.5/Rad2Deg;
    A1 = 20./Rad2Deg;
    dA = A1-A0;
    L0 = 0.05/Rad2Deg*Sphere::radius;
    L1 = 0.1/Rad2Deg*Sphere::radius;
    dL = L1-L0;
    REPORT_ONLINE("TTS")
}

TTS::~TTS()
{
    REPORT_OFFLINE("TTS")
}

inline void TTS::changeEdgeEndPoint(Edge *edge, PointOrder order, Vertex *point,
                                    MeshManager &meshManager,
                                    const FlowManager &flowManager) const
{
    Vertex *testPoint = edge->getTestPoint();
    testPoint->Point::reinit();
    edge->linkEndPoint(order, point);
    edge->calcNormVector();
    edge->calcLength();
#ifdef DEBUG
    assert(!edge->getEdgePointer(EdgeLeft)->next->isWrongAngle());
    assert(!edge->getEdgePointer(EdgeRight)->next->isWrongAngle());
#endif
    edge->getEdgePointer(EdgeLeft)->resetAngle();
    edge->getEdgePointer(EdgeLeft)->next->resetAngle();
    edge->getEdgePointer(EdgeRight)->resetAngle();
    edge->getEdgePointer(EdgeRight)->next->resetAngle();
    Location loc;
    meshManager.checkLocation(testPoint->getCoordinate(), loc);
    testPoint->setLocation(loc);
    track(meshManager, flowManager, testPoint);
}

void TTS::advect(MeshManager &meshManager, const FlowManager &flowManager,
                 ParcelManager &parcelManager)
{
    // -------------------------------------------------------------------------
    // for short hand
    PolygonManager &polygonManager = parcelManager.polygonManager;
    Polygon *polygon;
    char fileName[30];

    // Note: This operation is very costly! Optimize it!
    meshManager.resetPointCounter();

    // -------------------------------------------------------------------------
    // check the location of each vertex at the first step
    if (TimeManager::isFirstStep()) {
        Vertex *vertex = polygonManager.vertices.front();
        for (int i = 0; i < polygonManager.vertices.size(); ++i) {
            Location loc;
            meshManager.checkLocation(vertex->getCoordinate(), loc);
            vertex->setLocation(loc);
            vertex = vertex->next;
        }
    }

    // -------------------------------------------------------------------------
    // advect vertices of each parcel (polygon)
    Vertex *vertex = polygonManager.vertices.front();
    for (int i = 0; i < polygonManager.vertices.size(); ++i) {
        track(meshManager, flowManager, vertex, vertex->getID());
        vertex = vertex->next;
    }

    // -------------------------------------------------------------------------
    // handle parcel-splitting and parcel-merging
    // Note: New vertices may be added by curvature guard.
    //sprintf(fileName, "debug_counters%5.5d.nc", TimeManager::getSteps());
    //meshManager.pointCounter.output(fileName);

    // -------------------------------------------------------------------------
    Edge *edge = polygonManager.edges.front();
    for (int i = 0; i < polygonManager.edges.size(); ++i) {
        edge->calcNormVector();
        edge->calcLength();
        edge = edge->next;
    }

#ifdef DEBUG_OUTPUT
    sprintf(fileName, "tts_test%5.5d_1.nc", TimeManager::getSteps());
    polygonManager.output(fileName);
#endif
    
    // -------------------------------------------------------------------------
    // calculate the angle
    polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int j = 0; j < polygon->edgePointers.size(); ++j) {
            edgePointer->calcAngle();
            edgePointer = edgePointer->next;
        }
        polygon = polygon->next;
    }

    // -------------------------------------------------------------------------
    // guard the curvature of each parcel (polygon)
    guardCurvature(meshManager, flowManager, polygonManager);

    // -------------------------------------------------------------------------
    // update physical quantities
#ifdef DEBUG
    double totalArea = 0.0;
#endif
    polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        polygon->calcArea();
#ifdef DEBUG
        totalArea += polygon->getArea();
#endif
        polygon = polygon->next;
    }
#ifdef DEBUG
    cout << "Total area bias: " << setw(30) << setprecision(16);
    cout << fabs(totalArea-4.0*PI*Sphere::radius2) << endl;
    assert(fabs(totalArea-4.0*PI*Sphere::radius2) < 1.0e-6);
#endif
}

void TTS::track(MeshManager &meshManager, const FlowManager &flowManager,
                Point *point, int ID) const
{
    const Coordinate &x0 = point->getCoordinate();
    const Location &loc0 = point->getLocation();
    Coordinate x1;
    Location loc1 = loc0;
    Velocity::Type type;
    Velocity v1, v2, v3, v4, v;
    double dt = TimeManager::getTimeStep();
    double dt05 = dt*0.5;

    // -------------------------------------------------------------------------
    // Note: Set the velocity type to handle the occasion that the point is
    //       acrossing from pole region to normal region, in which case "loc1"
    //       will be different from "loc0" on the value of "onPole".
    if (loc0.onPole) {
        type = Velocity::StereoPlane;
    } else {
        type = Velocity::LonLatSpace;
    }

    // -------------------------------------------------------------------------
    flowManager.getVelocity(x0, loc0, OldTimeLevel, v1, type);
    meshManager.move(x0, x1, v1, dt05, loc0);
    meshManager.checkLocation(x1, loc1);
    flowManager.getVelocity(x1, loc1, HalfTimeLevel, v2, type);

    // -------------------------------------------------------------------------
    meshManager.move(x0, x1, v2, dt05, loc0);
    meshManager.checkLocation(x1, loc1);
    flowManager.getVelocity(x1, loc1, HalfTimeLevel, v3, type);

    // -------------------------------------------------------------------------
    meshManager.move(x0, x1, v3, dt, loc0);
    meshManager.checkLocation(x1, loc1);
    flowManager.getVelocity(x1, loc1, NewTimeLevel, v4, type);

    // -------------------------------------------------------------------------
    v = (v1+v2*2.0+v3*2.0+v4)/6.0;
    meshManager.move(x0, x1, v, dt, loc0);
    if (ID != -1) {
        meshManager.checkLocation(x1, loc1, ID);
    } else {
        meshManager.checkLocation(x1, loc1);
    }

    // -------------------------------------------------------------------------
    point->setCoordinate(x1);
    point->setLocation(loc1);
}

inline double TTS::angleThreshold(Edge *edge)
{
    double length = edge->getLength();

    if (length > L0 && length < L1) {
        double t = 1.0-(length-L0)/dL;
        return dA*(4.0-3.0*t)*pow(t, 3.0)+A0;
    } else if (length <= L0) {
        return A1;
    } else {
        return A0;
    }
}

inline double TTS::angleThreshold(Edge *edge1, Edge *edge2)
{
    double A1 = angleThreshold(edge1);
    double A2 = angleThreshold(edge2);
    return (A1+A2)*0.5;
}

void TTS::guardCurvature(MeshManager &meshManager, const FlowManager &flowManager,
                         PolygonManager &polygonManager)
{
    Edge *edge;
    Polygon *polygon;
    int numEdge;
    bool flag = false;

    // -------------------------------------------------------------------------
    // check the location of each edge's test point at the first step
    if (TimeManager::isFirstStep()) {
        edge = polygonManager.edges.front();
        for (int i = 0; i < polygonManager.edges.size(); ++i) {
            Vertex *testPoint = edge->getTestPoint();
            Location loc;
            meshManager.checkLocation(testPoint->getCoordinate(), loc);
            testPoint->setLocation(loc);
            edge = edge->next;
        }
    }

    // -------------------------------------------------------------------------
    // advect the test point
    edge = polygonManager.edges.front();
    for (int i = 0; i < polygonManager.edges.size(); ++i) {
        track(meshManager, flowManager, edge->getTestPoint());
        edge = edge->next;
    }

#ifdef DEBUG_OUTPUT
    char fileName[30];
    sprintf(fileName, "tts_test%5.5d_2.nc", TimeManager::getSteps());
    polygonManager.output(fileName);
#endif

    // -------------------------------------------------------------------------
    // edge-splitting
    numEdge = polygonManager.edges.size();
    edge = polygonManager.edges.front();
    for (int i = 0; i < numEdge; ++i) {
        splitEdge(meshManager, flowManager, polygonManager, edge);
        edge = edge->next;
    }

    numEdge = polygonManager.edges.size();
    edge = polygonManager.edges.front();
    for (int i = 0; i < numEdge; ++i) {
        if(handleSmallEdge(meshManager, flowManager, polygonManager, edge))
            flag = true;
        edge = edge->next;
    }

#ifdef DEBUG_OUTPUT
    sprintf(fileName, "tts_test%5.5d_3.nc", TimeManager::getSteps());
    polygonManager.output(fileName);
#endif

    // -------------------------------------------------------------------------
    // edge-merging
    polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        if (mergeEdge(meshManager, flowManager, polygonManager, polygon))
            flag = true;
        polygon = polygon->next;
    }

    // -------------------------------------------------------------------------
    // handle wrong angle
    polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        if (handleWrongAngle(meshManager, flowManager, polygonManager, polygon))
            flag = true;
        polygon = polygon->next;
    }

    // -------------------------------------------------------------------------
    // reindex the vertices and edges for outputting
    if (flag) {
        polygonManager.vertices.reindex();
        polygonManager.edges.reindex();
    }
}

#include "splitEdge.h"
#include "handleSmallEdge.h"
#include "mergeEdge.h"
#include "handleWrongAngle.h"
