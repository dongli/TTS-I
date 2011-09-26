#include "TTS.h"
#include "Velocity.h"
#include "Point.h"
#include "TimeManager.h"
#include "Constants.h"
#include "Sphere.h"
#include "CurvatureGuard.h"
#ifdef DEBUG
#include "DebugTools.h"
#endif

std::list<EdgePointer *> TTS::needUpdateAngles;

TTS::TTS()
{
    REPORT_ONLINE("TTS")
}

TTS::~TTS()
{
    REPORT_OFFLINE("TTS")
}

void TTS::advect(MeshManager &meshManager, const FlowManager &flowManager,
                 TracerManager &tracerManager)
{
    // -------------------------------------------------------------------------
    // for short hand
    PolygonManager &polygonManager = tracerManager.polygonManager;
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
    CurvatureGuard::guard(meshManager, flowManager, polygonManager);

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
    cout << "Total edge number: " << setw(10);
    cout << polygonManager.edges.size() << endl;
    cout << "Total polygon number: " << setw(10);
    cout << polygonManager.polygons.size() << endl;
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
            //dumpTask(type);
            REPORT_WARNING("The edge pointer does not contained in the list.")
        }
#endif
        needUpdateAngles.remove(edgePointer);
    }
}

void TTS::doTask(TaskType type, bool debug)
{
    if (type == UpdateAngle) {
        std::list<EdgePointer *>::iterator it = needUpdateAngles.begin();
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

void TTS::dumpTask(TaskType type)
{
    std::list<EdgePointer *>::const_iterator it;
    it = needUpdateAngles.begin();
    cout << setw(8) << "Edge ID";
    cout << setw(8) << "Orient";
    cout << setw(8) << "ID";
    cout << setw(20) << "First point ID";
    cout << setw(20) << "Second point ID" << endl;
    for (; it != needUpdateAngles.end(); ++it) {
        cout << setw(8) << (*it)->edge->getID();
        if ((*it)->orient == OrientLeft)
            cout << setw(8) << "left";
        else
            cout << setw(8) << "right";
        cout << setw(8) << (*it)->getID();
        cout << setw(20) << (*it)->getEndPoint(FirstPoint)->getID();
        cout << setw(20) << (*it)->getEndPoint(SecondPoint)->getID();
        cout << endl;
    }
}