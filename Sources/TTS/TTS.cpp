#include "TTS.h"
#include "Velocity.h"
#include "Point.h"
#include "TimeManager.h"
#include "Constants.h"
#include "Sphere.h"
#ifdef DEBUG
#include "DebugTools.h"
#endif
#include "splitEdge.h"
#include "mergeEdge.h"
#include "splitPolygon.h"

list<EdgePointer *> TTS::needUpdateAngles;

TTS::TTS()
{
    REPORT_ONLINE("TTS")
}

TTS::~TTS()
{
    REPORT_OFFLINE("TTS")
}

void TTS::init()
{
    A0 = 0.1/Rad2Deg;
    A1 = 60.0/Rad2Deg;
    dA = A1-A0;
    L0 = 0.01/Rad2Deg*Sphere::radius;
    L1 = 1.0/Rad2Deg*Sphere::radius;
    dL = L1-L0;
}

void TTS::advect(MeshManager &meshManager, const FlowManager &flowManager,
                 ParcelManager &parcelManager)
{
    // -------------------------------------------------------------------------
    // for short hand
    PolygonManager &polygonManager = parcelManager.polygonManager;
    Polygon *polygon;
    Vertex *vertex;
    Edge *edge;
    //char fileName[30];

    meshManager.resetPointCounter();

    // -------------------------------------------------------------------------
    // check the location of each vertex at the first step
    if (TimeManager::isFirstStep()) {
        vertex = polygonManager.vertices.front();
        for (int i = 0; i < polygonManager.vertices.size(); ++i) {
            Location loc;
            meshManager.checkLocation(vertex->getCoordinate(), loc);
            vertex->setLocation(loc);
            vertex = vertex->next;
        }
    }

    // -------------------------------------------------------------------------
    // advect vertices of each parcel (polygon)
    vertex = polygonManager.vertices.front();
    for (int i = 0; i < polygonManager.vertices.size(); ++i) {
        track(meshManager, flowManager, vertex, true);
        vertex = vertex->next;
    }
    //sprintf(fileName, "debug_counters%5.5d.nc", TimeManager::getSteps());
    //meshManager.pointCounter.output(fileName);

    // -------------------------------------------------------------------------
    edge = polygonManager.edges.front();
    for (int i = 0; i < polygonManager.edges.size(); ++i) {
        edge->calcNormVector();
        edge->calcLength();
        edge = edge->next;
    }

#ifdef DEBUG_BACKUP
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
    static const double trueTotalArea = 4.0*PI*Sphere::radius2;
    double bias = fabs(totalArea-trueTotalArea)/trueTotalArea;
    cout << "Total area bias: " << setw(30) << setprecision(16);
    cout << bias << endl;
    assert(bias < 1.0e-6);
#endif
}

void TTS::track(MeshManager &meshManager, const FlowManager &flowManager,
                Point *point, bool isCount)
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
    if (isCount) {
        meshManager.checkLocation(x1, loc1, point);
    } else {
        meshManager.checkLocation(x1, loc1);
    }

    // -------------------------------------------------------------------------
    point->setCoordinate(x1);
    point->setLocation(loc1);
}

void TTS::resetTasks()
{
    needUpdateAngles.clear();
}

void TTS::recordTask(TaskType type, EdgePointer *edgePointer)
{
    if (type == UpdateAngle) {
        if (find(needUpdateAngles.begin(), needUpdateAngles.end(), edgePointer)
            == needUpdateAngles.end()) {
            edgePointer->resetAngle();
            needUpdateAngles.push_back(edgePointer);
        }
    }
}

void TTS::deleteTask(TaskType type, EdgePointer *edgePointer)
{
    if (type == UpdateAngle) {
#ifdef DEBUG
        if (find(needUpdateAngles.begin(), needUpdateAngles.end(), edgePointer)
            == needUpdateAngles.end()) {
            REPORT_ERROR("The edge pointer does not contained in the list.")
        }
#endif
        needUpdateAngles.remove(edgePointer);
    }
}

void TTS::doTask(TaskType type, bool debug)
{
    if (type == UpdateAngle) {
        list<EdgePointer *>::iterator it = needUpdateAngles.begin();
        for (; it != needUpdateAngles.end(); ++it) {
#ifdef DEBUG
            assert((*it)->isAngleSet == false);
#endif
            (*it)->calcAngle();
            if (debug) {
                (*it)->dump();
            }
        }
        needUpdateAngles.clear();
    }
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
    Edge *edge, *nextEdge, *endEdge;
    bool flag = false;

    // -------------------------------------------------------------------------
    // some operations at the first step
    if (TimeManager::isFirstStep()) {
        // check the location of each edge's test point
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
    // advect test points
    edge = polygonManager.edges.front();
    for (int i = 0; i < polygonManager.edges.size(); ++i) {
        track(meshManager, flowManager, edge->getTestPoint());
        edge = edge->next;
    }

    // -------------------------------------------------------------------------
    // extract the vertex-edge approaching information
    ApproachDetector::detect(meshManager, flowManager, polygonManager);
    DebugTools::dump_watchers();

    // -------------------------------------------------------------------------
    // split polygons
    if (splitPolygon(meshManager, flowManager, polygonManager))
        flag = true;
    DebugTools::dump_watchers();

    // -------------------------------------------------------------------------
    // edge-splitting
    edge = polygonManager.edges.front();
    endEdge = polygonManager.edges.back();
    while (edge != endEdge->next) {
        nextEdge = edge->next;
        splitEdge(meshManager, flowManager, polygonManager, edge);
        edge = nextEdge;
    }
    DebugTools::dump_watchers();

    // -------------------------------------------------------------------------
    // edge-merging
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        if (mergeEdge(meshManager, flowManager, polygonManager, polygon))
            flag = true;
        polygon = polygon->next;
    }
    DebugTools::dump_watchers();

    // -------------------------------------------------------------------------
    // extract the vertex-edge approaching information
    ApproachDetector::detect(meshManager, flowManager, polygonManager);
    DebugTools::dump_watchers();

    // -------------------------------------------------------------------------
    // split polygons
    if (splitPolygon(meshManager, flowManager, polygonManager))
        flag = true;
    DebugTools::dump_watchers();

    // -------------------------------------------------------------------------
    // reindex the vertices and edges for outputting
    if (flag) {
        polygonManager.vertices.reindex();
        polygonManager.edges.reindex();
    }
}
